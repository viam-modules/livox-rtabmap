package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"path/filepath"
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
	DataType      string    `json:"data_type"` // "pcd" or "imu"
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
	imuComponent := flag.String("imu", os.Getenv("IMU_COMPONENT"), "IMU/movement-sensor component name (optional) [env: IMU_COMPONENT]")
	outDir := flag.String("out", outDefault, "output directory [env: OUT_DIR]")
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
	pcdEntries := downloadBinaryData(ctx, dataClient, &pcdFilter, pcdDir, "pcd", logger)
	fmt.Printf("downloaded %d PCD file(s) → %s\n", len(pcdEntries), pcdDir)
	allEntries = append(allEntries, pcdEntries...)

	// --- Download IMU tabular data (as JSON) ---
	if *imuComponent != "" {
		if err := os.MkdirAll(imuDir, 0o755); err != nil {
			logger.Fatalf("failed to create imu dir: %v", err)
		}

		imuFilter := baseFilter
		imuFilter.ComponentName = *imuComponent

		fmt.Printf("\nsearching for IMU data from component %q...\n", *imuComponent)
		imuEntries := downloadTabularData(ctx, dataClient, &imuFilter, imuDir, logger)
		fmt.Printf("downloaded %d IMU record(s) → %s\n", len(imuEntries), imuDir)
		allEntries = append(allEntries, imuEntries...)
	}

	if len(allEntries) > 0 {
		writeManifest(*outDir, allEntries, logger)
	}
}

// collectBinaryMetadata pages through BinaryDataByFilter (no binary payload) and returns IDs + stub entries.
func collectBinaryMetadata(ctx context.Context, dc *app.DataClient, filter *app.Filter, logger logging.Logger) ([]string, []ManifestEntry) {
	var ids []string
	var entries []ManifestEntry
	var last string

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
			ids = append(ids, d.Metadata.BinaryDataID)

			var component string
			var tags []string
			if d.Metadata != nil {
				component = d.Metadata.CaptureMetadata.ComponentName
				tags = d.Metadata.CaptureMetadata.Tags
			}

			entries = append(entries, ManifestEntry{
				ComponentName: component,
				TimeRequested: d.Metadata.TimeRequested,
				TimestampNs:   d.Metadata.TimeRequested.UnixNano(),
				Tags:          tags,
			})
		}

		last = resp.Last
		if len(resp.BinaryData) == 0 || last == "" {
			break
		}
	}
	return ids, entries
}

// downloadBinaryData fetches binary files, writes them to outDir with sortable names, returns manifest entries.
func downloadBinaryData(ctx context.Context, dc *app.DataClient, filter *app.Filter, outDir, dataType string, logger logging.Logger) []ManifestEntry {
	ids, entries := collectBinaryMetadata(ctx, dc, filter, logger)
	if len(ids) == 0 {
		return nil
	}

	const batchSize = 50
	for i := 0; i < len(ids); i += batchSize {
		end := i + batchSize
		if end > len(ids) {
			end = len(ids)
		}
		batch := ids[i:end]

		results, err := dc.BinaryDataByIDs(ctx, batch)
		if err != nil {
			logger.Errorf("BinaryDataByIDs (batch %d-%d) failed: %v", i, end-1, err)
			continue
		}

		for j, d := range results {
			idx := i + j
			component := entries[idx].ComponentName
			if component == "" {
				component = "lidar"
			}
			// Epoch-ns prefix ensures files sort chronologically
			filename := fmt.Sprintf("%d_%s.pcd", d.Metadata.TimeRequested.UnixNano(), component)
			dest := filepath.Join(outDir, filename)

			if err := os.WriteFile(dest, d.Binary, 0o644); err != nil {
				logger.Errorf("failed to write %s: %v", dest, err)
				continue
			}
			fmt.Printf("  -> %s\n", dest)
			entries[idx].Filename = filename
			entries[idx].DataType = dataType
		}
	}

	return entries
}

// downloadTabularData fetches tabular sensor data (e.g. IMU) and saves each record as a JSON file.
func downloadTabularData(ctx context.Context, dc *app.DataClient, filter *app.Filter, outDir string, logger logging.Logger) []ManifestEntry {
	var entries []ManifestEntry
	var last string

	for {
		resp, err := dc.TabularDataByFilter(ctx, &app.DataByFilterOptions{
			Filter: filter,
			Limit:  100,
			Last:   last,
		})
		if err != nil {
			logger.Errorf("TabularDataByFilter failed: %v", err)
			break
		}

		for _, d := range resp.TabularData {
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

			// Include method in filename to avoid collisions when multiple
			// measurement types share the same millisecond timestamp.
			filename := fmt.Sprintf("%d_%s_%s.json", d.TimeRequested.UnixNano(), component, method)
			dest := filepath.Join(outDir, filename)

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
			if err := os.WriteFile(dest, raw, 0o644); err != nil {
				logger.Errorf("failed to write %s: %v", dest, err)
				continue
			}
			fmt.Printf("  -> %s\n", dest)

			entries = append(entries, ManifestEntry{
				Filename:      filename,
				TimestampNs:   d.TimeRequested.UnixNano(),
				ComponentName: component,
				TimeRequested: d.TimeRequested,
				Tags:          tags,
				DataType:      "imu",
			})
		}

		last = resp.Last
		if len(resp.TabularData) == 0 || last == "" {
			break
		}
	}

	return entries
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
