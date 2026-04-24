package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/app"
	"go.viam.com/rdk/logging"
)

// ManifestEntry describes one downloaded file.
type ManifestEntry struct {
	Filename      string    `json:"filename"`
	TimestampNs   int64     `json:"timestamp_ns"`
	ComponentName string    `json:"component_name"`
	TimeRequested time.Time `json:"time_requested"`
	Tags          []string  `json:"tags"`
	DataType      string    `json:"data_type"` // "pcd", "imu", "color", or "depth"
}

func main() {
	apiKey := flag.String("api-key", os.Getenv("VIAM_API_KEY"), "Viam API key (required) [env: VIAM_API_KEY]")
	apiKeyID := flag.String("api-key-id", os.Getenv("VIAM_API_KEY_ID"), "Viam API key ID (required) [env: VIAM_API_KEY_ID]")
	orgID := flag.String("org-id", os.Getenv("VIAM_ORG_ID"), "Viam organization ID (required) [env: VIAM_ORG_ID]")
	locationID := flag.String("location-id", os.Getenv("VIAM_LOCATION_ID"), "Viam location ID (optional) [env: VIAM_LOCATION_ID]")
	machineID := flag.String("machine-id", os.Getenv("VIAM_MACHINE_ID"), "Viam machine/robot ID (optional) [env: VIAM_MACHINE_ID]")
	partID := flag.String("part-id", os.Getenv("VIAM_PART_ID"), "Viam robot part ID (optional) [env: VIAM_PART_ID]")
	outDefault := os.Getenv("OUT_DIR")
	if outDefault == "" {
		outDefault = "data"
	}
	since := flag.String("since", os.Getenv("SINCE"), "how far back to search, e.g. 24h, 7d, 2h30m [env: SINCE]")
	startStr := flag.String("start", os.Getenv("START"), "start of time range, e.g. 2025-03-20 or 2025-03-20T14:00:00Z [env: START]")
	endStr := flag.String("end", os.Getenv("END"), "end of time range (default: now) [env: END]")
	tag := flag.String("tag", os.Getenv("TAG"), "tag filter (optional) [env: TAG]")
	lidarComponent := flag.String("lidar", os.Getenv("LIDAR_COMPONENT"), "lidar component name (e.g. 'mid360') [env: LIDAR_COMPONENT]")
	imuComponent   := flag.String("imu", os.Getenv("IMU_COMPONENT"), "IMU/movement-sensor component name (optional) [env: IMU_COMPONENT]")
	rgbComponent   := flag.String("rgb-camera", os.Getenv("RGB_COMPONENT"), "RGBD camera component name (optional) [env: RGB_COMPONENT]")
	outDir    := flag.String("out", outDefault, "output directory [env: OUT_DIR]")
	workers   := flag.Int("workers", 4, "parallel download workers for binary data")
	flag.Parse()

	if *apiKey == "" || *apiKeyID == "" || *orgID == "" {
		fmt.Fprintln(os.Stderr, "usage: fetch-data --api-key=KEY --api-key-id=ID --org-id=ORG [options]")
		flag.PrintDefaults()
		os.Exit(1)
	}

	sinceSet := *since != ""
	startEndSet := *startStr != "" || *endStr != ""
	if sinceSet && startEndSet {
		fmt.Fprintln(os.Stderr, "error: --since and --start/--end are mutually exclusive")
		os.Exit(1)
	}
	if !sinceSet && !startEndSet {
		fmt.Fprintln(os.Stderr, "error: specify a time range with --since or --start/--end")
		os.Exit(1)
	}

	ctx := context.Background()
	logger := logging.NewLogger("fetch-data")

	client, err := app.CreateViamClientWithAPIKey(ctx, app.Options{}, *apiKey, *apiKeyID, logger)
	if err != nil {
		logger.Fatalf("failed to create client: %v", err)
	}
	defer client.Close()

	dataClient := client.DataClient()

	now := time.Now()
	var start, end time.Time

	if sinceSet {
		dur, err := parseDuration(*since)
		if err != nil {
			fmt.Fprintf(os.Stderr, "error: invalid --since %q: %v\n", *since, err)
			os.Exit(1)
		}
		start = now.Add(-dur)
		end = now
	} else {
		start, err = parseTime(*startStr)
		if err != nil {
			fmt.Fprintf(os.Stderr, "error: invalid --start %q: %v\n", *startStr, err)
			os.Exit(1)
		}
		if *endStr != "" {
			end, err = parseTime(*endStr)
			if err != nil {
				fmt.Fprintf(os.Stderr, "error: invalid --end %q: %v\n", *endStr, err)
				os.Exit(1)
			}
		} else {
			end = now
		}
	}

	baseFilter := app.Filter{
		OrganizationIDs: []string{*orgID},
		Interval: app.CaptureInterval{
			Start: start,
			End:   end,
		},
	}
	if *locationID != "" {
		baseFilter.LocationIDs = []string{*locationID}
	}
	if *machineID != "" {
		baseFilter.RobotID = *machineID
	}
	if *partID != "" {
		baseFilter.PartID = *partID
	}
	if *tag != "" {
		baseFilter.TagsFilter = app.TagsFilter{
			Type: app.TagsFilterTypeMatchByOr,
			Tags: []string{*tag},
		}
	}

	pcdDir := filepath.Join(*outDir, "pcd")
	imuDir := filepath.Join(*outDir, "imu")
	rgbDir := filepath.Join(*outDir, "rgb")

	if err := os.MkdirAll(pcdDir, 0o755); err != nil {
		logger.Fatalf("failed to create pcd dir: %v", err)
	}

	var allEntries []ManifestEntry

	// --- Download PCD binary data ---
	pcdFilter := baseFilter
	pcdFilter.MimeType = []string{"pointcloud/pcd"}
	if *lidarComponent != "" {
		pcdFilter.ComponentName = *lidarComponent
	}

	fmt.Printf("searching for PCD data from %s to %s...\n", start.Format(time.RFC3339), end.Format(time.RFC3339))
	pcdEntries := downloadBinaryData(ctx, dataClient, &pcdFilter, pcdDir, start, end, *workers, logger,
		func(ts int64, component, fileExt string) (string, string) {
			return fmt.Sprintf("%d_%s.pcd", ts, component), "pcd"
		})
	fmt.Printf("downloaded %d PCD file(s) → %s\n", len(pcdEntries), pcdDir)
	allEntries = append(allEntries, pcdEntries...)

	// --- Download RGBD image data ---
	if *rgbComponent != "" {
		if err := os.MkdirAll(rgbDir, 0o755); err != nil {
			logger.Fatalf("failed to create rgb dir: %v", err)
		}

		rgbFilter := baseFilter
		rgbFilter.ComponentName = *rgbComponent

		fmt.Printf("\nsearching for RGBD data from component %q...\n", *rgbComponent)
		rgbEntries := downloadBinaryData(ctx, dataClient, &rgbFilter, rgbDir, start, end, *workers, logger,
			func(ts int64, component, fileExt string) (string, string) {
				switch fileExt {
				case ".jpeg", ".jpg", ".png":
					return fmt.Sprintf("%d_color%s", ts, fileExt), "color"
				case ".dep":
					return fmt.Sprintf("%d_depth%s", ts, fileExt), "depth"
				default:
					return fmt.Sprintf("%d_%s%s", ts, component, fileExt), "color"
				}
			})
		fmt.Printf("downloaded %d RGBD image(s) → %s\n", len(rgbEntries), rgbDir)
		allEntries = append(allEntries, rgbEntries...)
	}

	// --- Download IMU tabular data (as JSON) ---
	if *imuComponent != "" {
		if err := os.MkdirAll(imuDir, 0o755); err != nil {
			logger.Fatalf("failed to create imu dir: %v", err)
		}

		imuFilter := baseFilter
		imuFilter.ComponentName = *imuComponent

		fmt.Printf("\nsearching for IMU data from component %q...\n", *imuComponent)
		imuEntries := downloadTabularData(ctx, dataClient, &imuFilter, imuDir, start, end, logger)
		fmt.Printf("downloaded %d IMU record(s) → %s\n", len(imuEntries), imuDir)
		allEntries = append(allEntries, imuEntries...)
	}

	if len(allEntries) > 0 {
		writeManifest(*outDir, allEntries, logger)
	}
}

// nameFn maps (timestamp_ns, component, fileExt) → (filename, dataType).
// fileExt includes the leading dot (e.g. ".pcd", ".jpeg", ".dep").
type nameFn func(ts int64, component, fileExt string) (filename, dataType string)

// collectBinaryMetadata pages through BinaryDataByFilter (no binary payload).
// Files already present in outDir are returned as cachedEntries (no download needed).
// Files that need downloading are returned as ids + downloadEntries (1:1 correspondence).
// Records outside [start, end] are skipped with a warning.
func collectBinaryMetadata(ctx context.Context, dc *app.DataClient, filter *app.Filter,
	outDir string, start, end time.Time, namer nameFn, logger logging.Logger,
) (ids []string, downloadEntries []ManifestEntry, cachedEntries []ManifestEntry) {
	var last string
	skipped := 0

	for {
		resp, err := dc.BinaryDataByFilter(ctx, false, &app.DataByFilterOptions{
			Filter: filter,
			Limit:  100,
			Last:   last,
		})
		if err != nil {
			logger.Fatalf("BinaryDataByFilter failed: %v", err)
		}

		for _, d := range resp.BinaryData {
			t := d.Metadata.TimeRequested
			if t.Before(start) || t.After(end) {
				fmt.Fprintf(os.Stderr, "  warning: skipping record outside time range: %s\n", t.Format(time.RFC3339))
				skipped++
				continue
			}

			var component, fileExt string
			var tags []string
			if d.Metadata != nil {
				component = d.Metadata.CaptureMetadata.ComponentName
				tags = d.Metadata.CaptureMetadata.Tags
				fileExt = d.Metadata.FileExt
			}
			if component == "" {
				component = "unknown"
			}

			filename, dataType := namer(t.UnixNano(), component, fileExt)
			entry := ManifestEntry{
				Filename:      filename,
				ComponentName: component,
				TimeRequested: t,
				TimestampNs:   t.UnixNano(),
				Tags:          tags,
				DataType:      dataType,
			}

			if _, err := os.Stat(filepath.Join(outDir, filename)); err == nil {
				cachedEntries = append(cachedEntries, entry)
			} else {
				ids = append(ids, d.Metadata.BinaryDataID)
				downloadEntries = append(downloadEntries, entry)
			}
		}

		last = resp.Last
		if len(resp.BinaryData) == 0 || last == "" {
			break
		}
	}
	if skipped > 0 {
		fmt.Fprintf(os.Stderr, "  warning: skipped %d record(s) outside [%s, %s]\n",
			skipped, start.Format(time.RFC3339), end.Format(time.RFC3339))
	}
	return
}

// downloadBinaryData fetches binary files in parallel, writes them to outDir, returns manifest entries.
func downloadBinaryData(ctx context.Context, dc *app.DataClient, filter *app.Filter, outDir string, start, end time.Time, numWorkers int, logger logging.Logger, namer nameFn) []ManifestEntry {
	ids, downloadEntries, cachedEntries := collectBinaryMetadata(ctx, dc, filter, outDir, start, end, namer, logger)

	total := len(ids) + len(cachedEntries)
	if total == 0 {
		return nil
	}
	if len(ids) == 0 {
		fmt.Printf("  %d/%d already cached\n", len(cachedEntries), total)
		return cachedEntries
	}

	const batchSize = 50

	type batch struct {
		startIdx int
		ids      []string
	}

	var batches []batch
	for i := 0; i < len(ids); i += batchSize {
		end := i + batchSize
		if end > len(ids) {
			end = len(ids)
		}
		batches = append(batches, batch{i, ids[i:end]})
	}

	var done atomic.Int64
	var mu sync.Mutex
	var wg sync.WaitGroup
	sem := make(chan struct{}, numWorkers)
	startTime := time.Now()

	nCached := len(cachedEntries)
	nDownload := len(ids)

	printProgress := func() {
		n := int(done.Load()) + nCached
		elapsed := time.Since(startTime).Round(time.Second)
		pct := n * 100 / total
		width := 30
		filled := width * n / total
		bar := strings.Repeat("#", filled) + strings.Repeat(".", width-filled)
		eta := ""
		nd := int(done.Load())
		if nd > 0 && nd < nDownload {
			remaining := time.Duration(float64(elapsed) * float64(nDownload-nd) / float64(nd))
			eta = fmt.Sprintf(" | eta %s", remaining.Round(time.Second))
		}
		cached_str := ""
		if nCached > 0 {
			cached_str = fmt.Sprintf(" (%d cached)", nCached)
		}
		fmt.Printf("\r  [%s] %d/%d%s  %d%%%s%s  ", bar, n, total, cached_str, pct, eta, strings.Repeat(" ", 10))
	}

	for _, b := range batches {
		wg.Add(1)
		sem <- struct{}{}
		go func(b batch) {
			defer wg.Done()
			defer func() { <-sem }()

			results, err := dc.BinaryDataByIDs(ctx, b.ids)
			if err != nil {
				logger.Errorf("BinaryDataByIDs (batch starting %d) failed: %v", b.startIdx, err)
				return
			}

			for j := range results {
				idx := b.startIdx + j
				dest := filepath.Join(outDir, downloadEntries[idx].Filename)
				if err := os.WriteFile(dest, results[j].Binary, 0o644); err != nil {
					logger.Errorf("failed to write %s: %v", dest, err)
				}
				done.Add(1)
				mu.Lock()
				printProgress()
				mu.Unlock()
			}
		}(b)
	}

	wg.Wait()
	elapsed := time.Since(startTime).Round(time.Millisecond)
	cached_str := ""
	if nCached > 0 {
		cached_str = fmt.Sprintf(" (%d cached)", nCached)
	}
	fmt.Printf("\r  [%s] %d/%d%s  done in %s%s\n",
		strings.Repeat("#", 30), total, total, cached_str, elapsed, strings.Repeat(" ", 20))
	return append(cachedEntries, downloadEntries...)
}

// downloadTabularData fetches tabular sensor data (e.g. IMU) and saves each record as a JSON file.
// Records whose TimeRequested falls outside [start, end] are skipped with a warning.
func downloadTabularData(ctx context.Context, dc *app.DataClient, filter *app.Filter, outDir string, start, end time.Time, logger logging.Logger) []ManifestEntry {
	// One lightweight count-only request to get the total so we can show a progress bar.
	countResp, err := dc.TabularDataByFilter(ctx, &app.DataByFilterOptions{
		Filter:    filter,
		CountOnly: true,
	})
	total := 0
	if err != nil {
		logger.Warnf("could not get tabular record count: %v — progress %% unavailable", err)
	} else {
		total = countResp.Count
	}

	type writeJob struct {
		filename string
		entry    ManifestEntry
		raw      []byte
	}

	const numWriters = 8
	jobs := make(chan writeJob, numWriters*4)

	var done atomic.Int64
	var cached atomic.Int64
	var mu sync.Mutex
	var allEntries []ManifestEntry
	startTime := time.Now()

	printProgress := func() {
		n := int(done.Load())
		elapsed := time.Since(startTime).Round(time.Second)
		c := int(cached.Load())
		cachedStr := ""
		if c > 0 {
			cachedStr = fmt.Sprintf(" (%d cached)", c)
		}
		if total > 0 {
			pct := n * 100 / total
			width := 30
			filled := width * n / total
			bar := strings.Repeat("#", filled) + strings.Repeat(".", width-filled)
			eta := ""
			if n > 0 && n < total {
				remaining := time.Duration(float64(elapsed) * float64(total-n) / float64(n))
				eta = fmt.Sprintf(" | eta %s", remaining.Round(time.Second))
			}
			fmt.Printf("\r  [%s] %d/%d%s  %d%%%s%s  ", bar, n, total, cachedStr, pct, eta, strings.Repeat(" ", 10))
		} else {
			fmt.Printf("\r  fetched %d records%s | %s elapsed%s", n, cachedStr, elapsed, strings.Repeat(" ", 10))
		}
	}

	// Writer pool: marshal+write in parallel while the main goroutine fetches pages.
	var wg sync.WaitGroup
	for i := 0; i < numWriters; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for job := range jobs {
				if _, err := os.Stat(filepath.Join(outDir, job.filename)); err == nil {
					cached.Add(1)
				} else if err := os.WriteFile(filepath.Join(outDir, job.filename), job.raw, 0o644); err != nil {
					logger.Errorf("failed to write %s: %v", job.filename, err)
				}
				done.Add(1)
				mu.Lock()
				allEntries = append(allEntries, job.entry)
				printProgress()
				mu.Unlock()
			}
		}()
	}

	// Main goroutine: fetch pages sequentially (cursor-based, can't parallelise),
	// send each record to the writer pool.
	var last string
	skipped := 0
	for {
		resp, err := dc.TabularDataByFilter(ctx, &app.DataByFilterOptions{
			Filter: filter,
			Limit:  1000,
			Last:   last,
		})
		if err != nil {
			logger.Errorf("TabularDataByFilter failed: %v", err)
			break
		}

		for _, d := range resp.TabularData {
			if d.TimeRequested.Before(start) || d.TimeRequested.After(end) {
				fmt.Fprintf(os.Stderr, "  warning: skipping record outside time range: %s\n", d.TimeRequested.Format(time.RFC3339))
				skipped++
				continue
			}
			var component, method string
			var tags []string
			if d.Metadata != nil {
				component = d.Metadata.ComponentName
				method = d.Metadata.MethodName
				tags = d.Metadata.Tags
			}
			if component == "" {
				component = "imu"
			}

			filename := fmt.Sprintf("%d_%s_%s.json", d.TimeRequested.UnixNano(), component, method)
			raw, err := json.Marshal(map[string]interface{}{
				"time_requested": d.TimeRequested,
				"time_received":  d.TimeReceived,
				"component":      component,
				"method":         method,
				"data":           d.Data,
			})
			if err != nil {
				logger.Errorf("failed to marshal record %s: %v", filename, err)
				continue
			}
			jobs <- writeJob{
				filename: filename,
				raw:      raw,
				entry: ManifestEntry{
					Filename:      filename,
					TimestampNs:   d.TimeRequested.UnixNano(),
					ComponentName: component,
					TimeRequested: d.TimeRequested,
					Tags:          tags,
					DataType:      "imu",
				},
			}
		}

		last = resp.Last
		if len(resp.TabularData) == 0 || last == "" {
			break
		}
	}
	close(jobs)
	wg.Wait()

	elapsed := time.Since(startTime).Round(time.Millisecond)
	n := int(done.Load())
	c := int(cached.Load())
	cachedStr := ""
	if c > 0 {
		cachedStr = fmt.Sprintf(" (%d cached)", c)
	}
	if total > 0 {
		fmt.Printf("\r  [%s] %d/%d%s  done in %s%s\n",
			strings.Repeat("#", 30), n, total, cachedStr, elapsed, strings.Repeat(" ", 20))
	} else {
		fmt.Printf("\r  fetched %d IMU records%s in %s%s\n",
			n, cachedStr, elapsed, strings.Repeat(" ", 20))
	}

	if skipped > 0 {
		fmt.Fprintf(os.Stderr, "  warning: skipped %d IMU record(s) outside [%s, %s]\n",
			skipped, start.Format(time.RFC3339), end.Format(time.RFC3339))
	}
	return allEntries
}

// parseDuration extends time.ParseDuration to support "d" for days (e.g. "7d", "2d12h").
func parseDuration(s string) (time.Duration, error) {
	// Replace trailing "d" suffix with equivalent hours so time.ParseDuration can handle it.
	// Supports formats like "7d", "2d12h", "1d30m".
	result := time.Duration(0)
	rest := s
	for rest != "" {
		var n int
		_, err := fmt.Sscanf(rest, "%d", &n)
		if err != nil {
			break
		}
		// find where the number ends
		i := 0
		for i < len(rest) && (rest[i] == '-' || (rest[i] >= '0' && rest[i] <= '9')) {
			i++
		}
		if i >= len(rest) {
			return 0, fmt.Errorf("missing unit in %q", s)
		}
		unit := rest[i]
		rest = rest[i+1:]
		switch unit {
		case 'd':
			result += time.Duration(n) * 24 * time.Hour
		default:
			// hand the remainder back to time.ParseDuration
			remaining := fmt.Sprintf("%d%c%s", n, unit, rest)
			d, err := time.ParseDuration(remaining)
			if err != nil {
				return 0, err
			}
			result += d
			rest = ""
		}
	}
	return result, nil
}

// parseTime parses a date or datetime string in common formats.
// Strings with explicit timezone info (RFC3339) are parsed as-is.
// Strings without timezone info are interpreted as local time.
func parseTime(s string) (time.Time, error) {
	// RFC3339 variants have timezone embedded — don't override it with local.
	for _, f := range []string{time.RFC3339Nano, time.RFC3339} {
		if t, err := time.Parse(f, s); err == nil {
			return t, nil
		}
	}
	// Everything else: no timezone in the string, assume local.
	for _, f := range []string{
		"2006-01-02T15:04:05",
		"2006-01-02 15:04:05",
		"2006-01-02",
	} {
		if t, err := time.ParseInLocation(f, s, time.Local); err == nil {
			return t, nil
		}
	}
	return time.Time{}, fmt.Errorf("unrecognized time format %q (try 2006-01-02 or 2006-01-02T15:04:05Z)", s)
}

func writeManifest(outDir string, entries []ManifestEntry, logger logging.Logger) {
	manifest := map[string]interface{}{
		"created_at": time.Now().UTC(),
		"files":      entries,
	}
	raw, err := json.MarshalIndent(manifest, "", "  ")
	if err != nil {
		logger.Errorf("failed to marshal manifest: %v", err)
		return
	}
	dest := filepath.Join(outDir, "manifest.json")
	if err := os.WriteFile(dest, raw, 0o644); err != nil {
		logger.Errorf("failed to write manifest: %v", err)
		return
	}
	fmt.Printf("\nmanifest written to %s\n", dest)
}
