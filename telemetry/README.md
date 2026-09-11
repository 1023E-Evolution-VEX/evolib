Telemetry is an opt-in library component. It does not modify or automatically hook `main.cpp`, autonomous, or driver control. Include `evolib/telemetry.hpp` (also exported by `evolib/api.hpp`) in any project using EvoLib.

Declare a persistent logger after your robot and choose your own signed motor ports and labels:

```cpp
evolib::Telemetry telemetry(robot, {
    {1, "left"}, {-2, "right"}, {3, "intake"}
});
```

Call these from your own setup and run lifecycle:

```cpp
telemetry.initialize();         // During setup, after odometry is ready.
telemetry.start("skills_run");  // Immediately before the route.
// Run the route; wait for its asynchronous motions to finish.
telemetry.stop();               // After the route, and when disabled/interrupted.
```

Initialize the worker in persistent program setup, rather than first creating it inside a competition task that may be killed. `start()` creates the worker if needed, but explicit initialization is preferred. Each `start()` requests a new file. Logging is not tied to any particular competition mode; call the same API for driver runs or mechanism tests. Stop requests do not wait for SD writes; poll `status()` until it is no longer `recording` or `opening` before removing the card.

Other odometry systems can supply a callback instead of an EvoLib Robot:

```cpp
evolib::Telemetry telemetry(
    [] { return evolib::Pose{xInches, yInches, headingDegrees}; },
    {{1, "left"}, {-2, "right"}}
);
```

Replace those variables with your own pose getter. The callback runs on the telemetry worker; it must read a thread-safe snapshot, return promptly, and keep captured objects alive. Use inches and clockwise degrees with zero along +Y. The Robot constructor supplies its `getPose()` callback automatically. No sensor ports are hard-coded in the logger.

Logs are written to `/usd/evo_000001.csv`, `/usd/evo_000002.csv`, etc.; existing readable files are skipped, including after a program restart. Use one logger per output directory.

The default is 20 Hz with a flush every second and a 32 MiB file limit. SD work runs in a separate lower-priority task and never runs inside motion controllers. Slow writes can skip samples, counted in `missed_samples`. The last buffered second may be lost on abrupt power removal; stop logging and let it finish before removing the card. A missing/full/removed card stops logging without stopping autonomous. A new `start()` attempts a new file.

`telemetry.status()` reports `idle`, `opening`, `recording`, `noCard`, `ioError`, `fileLimit`, or `invalidConfig`. `telemetry.filename()` returns the last successfully opened path. `start()` and `stop()` are asynchronous; file close finishes in the worker. Hardware objects and Robot must outlive the logger. Configure period, flush interval, and size through `TelemetryConfig`; periods below 20 ms are rejected.

Each CSV row contains:

- Sample number, monotonic Brain uptime in milliseconds, elapsed milliseconds/seconds, run label, and competition mode.
- Odometry X/Y in inches, heading in degrees (zero = +Y, clockwise positive), translational speed magnitude in inches/s, and angular speed in degrees/s.
- Battery millivolts and remaining capacity percent.
- Each configured motor's measured RPM, requested velocity RPM, temperature °C, applied millivolts, and current milliamps. Header names identify the physical port and configured label. Reversed port signs are retained when querying PROS. Invalid readings are blank rather than infinity/error sentinel values.

Timestamps are relative to Brain program uptime, not UTC calendar time. Speeds are differences between successive odometry samples; position resets/corrections can create spikes. Coordinates reflect the existing EvoLib odometry implementation, including its current tracking-wheel configuration. They are the estimated route, not an independent ground-truth measurement. Requested velocity is PROS's velocity-control target; the existing `move()` voltage commands should be inspected with the voltage panel.

The configured motor list belongs to the caller. Use the actual signed ports used by the code being logged, and choose stable labels. The library template exports the telemetry header and includes its implementation in `EvoLib.a`; consuming projects must build with C++20 or later for the bundled PROS headers.

To view runs locally:

1. Add the logger to your own project as above, insert a Brain-compatible FAT32 microSD card, and run code between `start()` and `stop()`. The logger uses the card root; no directory creation is needed.
2. After logging stops, copy the CSV files from the card into `telemetry/grafana/data/`. Keep original files; rename copies if files from different cards share a name.
3. From `telemetry/grafana/`, run `python generate.py --csv data/evo_000001.csv` (substitute your filename). This configures all motor plots from your CSV header. With Docker Compose installed, run `docker compose up -d` from the same folder.
4. Open http://localhost:3000, sign in with Grafana's initial `admin` / `admin` credentials, set a password, and open **EvoLib → EvoLib autonomous telemetry**.
5. Enter the copied filename, such as `evo_000001.csv`, in **CSV filename**. The default `example.csv` is explicitly synthetic demonstration data.

The dashboard is provisioned along with the Infinity data source. All plots use numeric XY axes, so the Grafana time picker does not filter these run files. The route uses X/Y inches; the other plots use elapsed seconds. The route's axis scales are independent. Rerun the generator with `--csv` when changing motor labels/ports; it changes the dashboard, never your recordings. For other runs with the same motor list, just change the dashboard's filename. Running the generator without arguments restores the bundled synthetic demonstration. Docker mounts recorded CSVs read-only, and Grafana is exposed only on localhost.

For an existing Grafana instance, install the Infinity data source, serve the CSV folder over HTTP where the Grafana server can reach it, create an Infinity source with UID `evo-sd`, and import `grafana/dashboards/telemetry.json`. Replace `http://csv/` in its query URLs with that server's URL.

Build with `pros make`. The Makefile uses C++20 supported by the installed toolchain and excludes two pre-existing unfinished motion sources that could not compile. Host logger tests: run `telemetry/tests/run.ps1` from a Visual Studio developer shell with Clang installed. Dashboard/CSV consistency: `python telemetry/tests/check_dashboard.py`.

Grafana references: [Infinity CSV](https://grafana.com/docs/plugins/yesoreyeram-infinity-datasource/latest/data-formats/csv/), [XY charts](https://grafana.com/docs/grafana/latest/visualizations/panels-visualizations/visualizations/xy-chart/), [data-source provisioning](https://grafana.com/docs/plugins/yesoreyeram-infinity-datasource/latest/configure/).
