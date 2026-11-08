# UtilityPi Dashboard

A home-monitoring dashboard served by a Raspberry Pi 5 (**utilitypi**) and shown on its touchscreen, on phones,
on the kitchen fridge and on any browser over Tailscale. One Python server (`dashboard_server.py`) collects
everything; one page (`index.html`) displays it as a swipeable carousel.

| Page | What it shows | Source |
|---|---|---|
| **Meters** | Electric (W), gas (ft³/h), water (gallons today); 24-hour clock charts and previous day | RTL-SDR radio (electric, gas), ESP32-CAM meter reader (water) |
| **Markets** | AAPL, NVDA, TSLA, AMD, ARM, USD/INR, plus a market-correlation line | Finnhub, frankfurter.dev |
| **Pi-hole** | Blocked %, queries, clients, 24 h query chart, top blocked domains | Pi-hole v6 API |
| **Driveway** | Live snapshot of the Reolink camera, full screen | go2rtc |
| **Cameras** | On-demand live view of 2 Eufy SoloCams and the Eufy doorbell | Eufy bridge + go2rtc |
| **Climate** | 2 Nest thermostats and their temperature sensors | Google Nest gateway |
| **Weather** | Current conditions, high/low, humidity, wind | Open-Meteo |

---

## System overview

```
                                   ┌──────────────────────── utilitypi (Raspberry Pi 5, Ethernet) ─────────────────────────┐
 Electric & gas meters ── 912 MHz ─┤                                                                                       │
   (ERT radio broadcasts)          │  dashboard_server.py  :8080 ──► index.html ──► Pi touchscreen (Chromium kiosk)        │
            │                      │    ├─ rtlamr ◄───────── rtl_tcp ◄── RTL-SDR dongle on the Pi-hole Pi (10.0.0.9)       │
            ▼                      │    ├─ water_fetcher ◄── ESP32-CAM water meter reader (10.0.0.241 /json)              │
   RTL-SDR on Pi-hole Pi ──────────┤    ├─ stock_fetcher ◄── Finnhub, frankfurter.dev (internet)                          │
                                   │    ├─ pihole_fetcher ◄─ Pi-hole v6 API (10.0.0.9)                                    │
                                   │    ├─ weather_fetcher ◄ Open-Meteo (internet)                                        │
                                   │    ├─ nest_fetcher ◄─── Google Nest gateway (internet, gRPC stream)                  │
                                   │    ├─ /api/camera.jpg ◄ go2rtc ◄── Reolink driveway camera (10.0.0.240, RTSP)         │
                                   │    └─ /api/cam-ws ────► go2rtc ◄── eufy_rtsp_bridge.py ◄── eufy-ws (Docker) ◄── Eufy  │
                                   │                         (~/Eufy, eufy-stream.service)                     cameras    │
                                   └───────────────────────────────────────────────────────────────────────────────────────┘
        Viewers:  Pi screen · http://utilitypi.local:8080 (LAN, phone, fridge) · https://utilitypi.tailebfffc.ts.net (Tailscale)
```

---

## Files

| File | Purpose |
|---|---|
| `dashboard_server.py` | HTTP server (port 8080) plus one background thread per data source |
| `index.html` | The whole front end: carousel, charts, camera player, layouts |
| `nest_google.py` | Nest thermostats via Google's gRPC gateway (see Climate) |
| `nest_proto.desc` | Compiled Nest protobuf definitions used by `nest_google.py` |
| `daily_data.json` | Persisted 24-hour rings for electric / gas / water (written on every reading) |
| `daily_history.csv` | One row per day of hourly usage, kept 90 days |
| `stock_history.csv` | Daily stock returns for the correlation line |
| `secrets.env` | API keys and passwords (**git-ignored**, `chmod 600`) |
| `nest_issue_token.txt`, `nest_cookie.txt` | Nest login (**git-ignored** as `nest_*.txt`, `chmod 600`) |

### HTTP API

| Endpoint | Returns |
|---|---|
| `/` | `index.html` |
| `/api/status` | Live electric/gas/water readings and rates, water camera status |
| `/api/hourly` | Today's 24 hourly buckets per meter |
| `/api/history` | Previous days' hourly buckets |
| `/api/stocks` | Quotes, `pca` correlation block |
| `/api/pihole` | Pi-hole stats, history, top blocked |
| `/api/weather` | Current weather |
| `/api/nest` | Thermostats and their sensors |
| `/api/camera.jpg` | Latest driveway snapshot |
| `/api/cam-ws?src=…` | WebSocket pass-through to go2rtc for the Eufy cameras (whitelisted stream names only) |

---

## Meters page

![Meters page](docs/screenshots/meters.png)


### Electric and gas: RTL-SDR

The utility meters broadcast their readings by radio (ERT, around 912 MHz). An RTL-SDR dongle on the **Pi-hole
Pi (10.0.0.9)** runs `rtl_tcp`; utilitypi decodes the stream with `rtlamr`:

```
rtlamr -server=10.0.0.9:1234 -filterid=<electric>,<gas> -msgtype=scm,scm+ -format=json -duration=1h
```

- **The dongle lives on the Pi-hole Pi**, not on utilitypi. Utilitypi's own WiFi was unreliable; the radio and
  the dashboard were split so either can move.
- **Only SCM / SCM+ messages are used.** IDM's consumption field lags SCM+ and made the series step backwards.
- **`plausible()` rejects impossible jumps** (bit-flip decodes, e.g. a +2^28 reading). Two consecutive agreeing
  readings re-baseline it.
- **Rate number:** averaged over the last 5 readings; it greys out after 5 min (electric) or 15 min (gas)
  without data.

Pi-hole side (systemd):
- `rtl_tcp.service`, with a drop-in that waits for an IP address before binding. Without it, `rtl_tcp` started
  before eth0 had its address after a reboot.
- A udev rule for the dongle and a DVB driver blacklist.
- `dashboard.service` does **not** `Require` rtl_tcp, so a radio outage can't take the dashboard down.

### Water: ESP32-CAM meter reader

An AI-Thinker ESP32-CAM running **AI-on-the-edge-device** (v16.1.0) photographs the ABB C-700 water meter in a
3D-printed shroud and reads the six drums and the sweep hand with neural networks.

- **Address:** `http://10.0.0.241` (static). It takes a reading every 5 minutes.
- **Polling:** the server polls `/json` once a minute and accepts a value only when the device reports
  `no error`. The device already rejects readings that go backwards or jump faster than `MaxRateValue`.
- **Units:** the meter counts cubic feet; the panel shows **gallons today** (× 7.48052) and the meter total.
- **Live view blocks the API:** while the device's live view is open in a browser, its small web server can't
  answer `/json`, and the panel greys out as "camera offline". Close the live view and it recovers on the
  next poll.
- **Accuracy:** reliable on still frames. While water is flowing, an occasional misread can be accepted and
  then block correct readings for a while. The total heals itself. The shroud's side-LED lighting is the
  planned improvement.

### Charts

- **24-spoke clock:** each spoke is an hour's usage, scaled to the busiest hour; the white hand is now. Slots
  roll over one hour at a time, so the ring never blanks at midnight.
- **Bars:** the previous complete day, from `daily_history.csv`.

---

## Markets page

![Markets page](docs/screenshots/markets.png)


- **Quotes:** Finnhub (`FINNHUB_KEY` in `secrets.env`) every 60 s, plus USD/INR from frankfurter.dev. The last
  good value is kept if a fetch fails.
- **Correlation line:** a rolling PCA over the five stocks' daily returns (last 60 trading days, needs at least
  10). The number is the share of their movement explained by one common factor:
  green below 50 % (moving independently), yellow 50–75 %, red 75 %+ (moving as one). With five stocks the floor
  is 20 %. The detail line shows the average pairwise correlation and the second factor's share.
- **Known flaw:** returns are recorded once per calendar day, so on weekends Friday's return is recorded again,
  three times in all. That over-weights Fridays.

---

## Pi-hole page

![Pi-hole page](docs/screenshots/pihole.png)


- **Source:** the Pi-hole v6 API at `http://10.0.0.9`, polled every 30 s with one reused session
  (`PIHOLE_PASSWORD` in `secrets.env`). Repeated bad logins back off to 5 min.
- **Shows:** blocked %, a split bar, queries and clients, a 24-hour chart (10-minute columns, allowed/blocked),
  and the top 5 blocked domains.
- **Related:** after a ~31-hour DNS outage, when the Pi-hole lost its default route after a gateway reboot, the
  Pi-hole gained `net-watchdog.service` / `.timer`, which restores networking, and persistent journald.

---

## Driveway page: Reolink RLC-510A

![Driveway page](docs/screenshots/driveway.png)


- **Camera:** static IP **10.0.0.240**, with RTSP enabled in the Reolink app (More Features ▸ Advanced Network
  Settings ▸ Server Settings).
- **go2rtc stream:** `driveway` uses the camera's sub stream
  (`rtsp://admin:${CAM_PASS}@10.0.0.240:554/h264Preview_01_sub`). `CAM_PASS` reaches go2rtc through a systemd
  drop-in (see Services).
- **Snapshots:** the server fetches `/api/frame.jpeg?src=driveway` from go2rtc on localhost and caches it for
  0.8 s, so several screens share one fetch. The page requests a frame every second, **only while the page is
  visible**.
- **Display:** the picture fills the page at any screen size (`object-fit: contain`). The sub stream is low
  resolution, so it looks soft when enlarged. The main stream would be sharper, but the Pi would have to decode
  it in software for every snapshot.

---

## Cameras page: Eufy, on demand

![Cameras page](docs/screenshots/cameras.png)


Two **Eufy SoloCams** (battery, H.265) and the **Eufy doorbell** (H.264). Nothing streams until a tile is tapped.
The batteries would otherwise drain in days.

```
 tap tile ─► index.html MSE player ─► wss /api/cam-ws?src=… ─► dashboard_server.py (whitelist, 10-min cap)
                                                                   │
                                                          go2rtc :1984 (~/Eufy/go2rtc.yaml)
                                                                   │ exec: ffmpeg reads tcp://127.0.0.1:900x
                                                     eufy_rtsp_bridge.py (on demand, ports 9001-9003)
                                                                   │ device.start_livestream / stop_livestream
                                                     eufy-ws (Docker, bropat/eufy-security-ws, :3000) ─► Eufy P2P
```

### Page behaviour (`index.html`)
- **Starting and stopping:** tap to start, tap again to stop.
- **Automatic stops:** after **2 min**, when the page is swiped or scrolled away for 1.5 s, or when the screen
  or tab is hidden.
- **One camera at a time:** `CAM_MAX_LIVE = 1`.
- **Waking:** a spinner and seconds counter while the camera wakes (about 4–6 s); it gives up after 30 s.
- **When stopped:** the last frame stays on the tile with "last seen" time.
- **Player:** go2rtc's MSE WebSocket protocol, reimplemented: it trims old buffered video, catches up gently,
  and never removes what's on screen.
- **SoloCams always use the H.264 copies** (`SOLOCAM_FORCE_H264`). Browsers decoding the H.265 themselves
  (notably the Pi's Chromium) showed green borders and crashed.

### Server pass-through (`dashboard_server.py`)
- **`/api/cam-ws` hands the WebSocket to go2rtc on localhost.** The same URL therefore works on the Pi, the
  phone and through Tailscale HTTPS. Only the five known stream names are accepted.
- **Hard cap:** 10 minutes per connection.

### Bridge (`~/Eufy`, its own git repo)
- **`eufy-stream.service`** runs `start_eufy_stack.sh`, which:
  - checks the `eufy-ws` container;
  - starts go2rtc with `~/Eufy/go2rtc.yaml`;
  - starts `eufy_rtsp_bridge.py`.
- **`eufy_rtsp_bridge.py` (rewritten on-demand):**
  - **Waking:** a camera's livestream starts only when go2rtc connects to its port, and stops **10 s** after the
    last viewer leaves.
  - **Caps:** at most **10 min** per livestream, followed by a **60 s** cooldown, because go2rtc retries
    instantly and would otherwise get round the cap.
  - **Clean joins:** it caches H.265/H.264 parameter sets and the frames since the last keyframe, so a late
    joiner never decodes green garbage.
  - **Isolation:** one camera's failure doesn't stop the others; the eufy-ws link reconnects with backoff.
  - **Shutdown:** puts streaming cameras back to sleep and refuses reconnects.
- **`go2rtc.yaml` streams:**
  - `solocam1/2`: H.265 handed over as **MPEG-TS on a pipe**. Over RTSP, their large keyframes (up to 170 KB)
    came out corrupted in go2rtc's browser output.
  - `solocam1_h264/2_h264`: converted by ffmpeg with
    `-x264-params sliced-threads=0:rc-lookahead=0:sync-lookahead=0`. **Do not use `-tune zerolatency`:** its
    multi-slice frames showed vertical green bands in the Pi's Chromium.
  - `doorbell`: H.264 copy.
  - `driveway`: the Reolink.
- **Network exposure:** go2rtc listens on `0.0.0.0` (1984/8554/8555) for the Tailscale link
  `http://100.68.193.42:1984/stream.html?src=…`. Anyone on the LAN can open a feed, which wakes a camera. The
  dashboard doesn't need it open.
- **The old `go2rtc.service`** (`/etc/go2rtc/go2rtc.yaml`) is **disabled**; the Eufy stack owns go2rtc now.

---

## Climate page: Nest thermostats and temperature sensors

![Climate page](docs/screenshots/climate.png)


Both Nest Learning Thermostats (3rd gen) and their Nest Temperature Sensors ("pucks"): current temperature,
setpoint, heating/cooling state, humidity, and each puck's reading. It is **read-only**: nothing here ever
changes a thermostat setting.

### How the data gets from the thermostat to the dashboard

```
 ┌──────────────┐  Bluetooth   ┌────────────────────┐   Wi-Fi, continuous   ┌──────────────────────────┐
 │ Nest pucks   │ ───────────► │ Nest thermostats   │ ────────────────────► │ Google / Nest cloud      │
 │ (temperature)│              │ (2× Learning Gen 3)│   (temps, setpoint,   │ (stores live device data)│
 └──────────────┘              └────────────────────┘    heating state)     └────────────┬─────────────┘
                                                                                         │ 4. Observe stream
                                                                                         │    (gRPC, live updates)
 utilitypi ─ dashboard_server.py                                                         │
 ┌───────────────────────────────────────────────────────────────────────────────────────┼────────────┐
 │                                                                                       │            │
 │  nest_cookie.txt ┐       1. "I'm logged in"          ┌───────────────────────┐        │            │
 │  nest_issue_     ├──────────────────────────────────►│ accounts.google.com   │        │            │
 │  token.txt       ┘◄──── Google access token (1 h) ───│                       │        │            │
 │        │                                             └───────────────────────┘        │            │
 │        │ 2. trade Google token ─────────────────────► Nest auth proxy ─► Nest JWT     │            │
 │        │ 3. trade JWT ──────────────────────────────► home.nest.com ──► session token │            │
 │        ▼                                                                              ▼            │
 │  nest_fetcher thread ── "Basic <session token>" ──► apigw.production.nest.com ──► nest_google.py   │
 │        │                                                              (decodes protobuf traits)    │
 │        ▼                                                                                           │
 │  /api/nest  (JSON: temps, setpoints, heating, pucks)                                               │
 └────────┬───────────────────────────────────────────────────────────────────────────────────────────┘
          │ every 60 s
          ▼
   index.html Climate page (Pi screen, phone, fridge, laptop)
```

- **Thermostat side.** The pucks talk to their thermostat over Bluetooth; the thermostats report to Google's cloud
  over Wi-Fi. The dashboard never talks to the thermostats directly (this model has no local API).
- **Why gRPC.** Since the account moved to Google, the thermostats are no longer in the old home.nest.com REST
  data (only the pucks are). They are only available through the streaming `Observe` call on
  `apigw.production.nest.com`, the one the Nest app uses. The stream stays open, so the page updates live; it is
  renewed before the token expires.
- **Where the method comes from.** The login chain and the Observe call follow the open-source
  [homebridge-nest-accfactory](https://www.npmjs.com/package/homebridge-nest-accfactory) plugin. Its `.proto`
  definitions are compiled into `nest_proto.desc`, so the Pi needs only `sudo apt install python3-grpcio
  python3-protobuf`. Without those, the server falls back to the REST data and shows only the pucks.
- **Cost:** none. This is not Google's registered/paid Device Access (SDM) API.
- **Display names:** override them in `NEST_RENAME` in `dashboard_server.py` (Nest's name → the dashboard's).
  It applies to thermostats and pucks. Nest uses a curly apostrophe (`’`) in names like "Ajith’s room".
- **Pucks:** both last reported in 2025, so their readings are greyed as stale. They take a **CR2** battery
  (twist the back off); re-pair them in the app if a new battery doesn't bring them back.

### The role of the Google cookie

The cookie (with the issue-token URL) is the **master key**, the only long-lived credential in the chain.

- **What it is:** what Safari had after signing in to home.nest.com with Google. It proves "already signed in":
  no password, no 2-step prompt.
- **What it's used for:** only to obtain short-lived tokens (steps 1–3). The Google token lasts about an hour, so
  the dashboard repeats steps 1–3 roughly hourly.
- **What breaks it:** logging out of home.nest.com, changing the Google password, or Google expiring the session.
  The page then shows *"Google login expired – capture a new cookie"*. The thermostats keep working; only the
  dashboard's view stops.
- **Why it's sensitive:** it is a signed-in Google session, not a Nest-only key. Never commit or share
  `nest_*.txt`.

### Re-capturing the login (when the cookie expires)

Must be done in **Safari**; other browsers give credentials that expire quickly.

1. Safari ▸ Settings ▸ Advanced ▸ *Show features for web developers*.
2. New **Private Window** ▸ Develop ▸ Show Web Inspector ▸ **Network**; tick *Preserve Log*; filter `issueToken`.
3. Go to https://home.nest.com and *Sign in with Google*.
4. Click the **iframerpc** request ▸ **Headers**:
   - **Summary ▸ URL** (`https://accounts.google.com/o/oauth2/iframerpc?action=issueToken…`) → `nest_issue_token.txt`
   - **Request ▸ Cookie** (one long line, must contain `SIDCC=`) → `nest_cookie.txt`
5. On the Pi: paste each into its file with `nano`, then
   `chmod 600 ~/dashboard/nest_*.txt && sudo systemctl restart dashboard`, and check with
   `curl -s localhost:8080/api/nest | head -c 600`.
6. Close the private window; **don't log out** of home.nest.com.

---

## Weather page

![Weather page](docs/screenshots/weather.png)


Open-Meteo (no key) for the house's coordinates (`WEATHER_LAT/LON`), refreshed every 15 min: temperature, feels-like,
high/low, humidity, wind, and a WMO weather-code description.

---

## Front end (`index.html`)

<img src="docs/screenshots/phone.png" alt="Stacked layout on a phone" width="300">


- **Carousel:** 7 pages in a ring, so the last page wraps to the first.
  - **Navigation:** swipe (touch and mouse), plus large ‹ › buttons that always work on the touch panel.
  - **Autocycle:** moves to the next page after 5 min without a touch.
- **Layouts:** side-by-side (`wide`) on landscape screens, and one long scrolling page (`stacked`) on portrait
  screens (phone, fridge).
  - **Forcing a layout:** add `?layout=stack` or `?layout=wide` to the URL.
  - **Scrolling:** the stacked layout scrolls in a loop.
- **Stale data:** a number turns grey when its source has gone quiet; each page shows "updated …" or an error.
- **Debug overlay:** add `#info` to the URL to show the viewport size, screen size and user agent.
- **Reloading the Pi's screen** after changing `index.html`: press F5, or
  `DISPLAY=:0 xdotool search --onlyvisible --class chromium windowactivate --sync key F5`, or
  `sudo systemctl restart display-manager`.

---

## Access

| From | URL |
|---|---|
| Pi touchscreen | Chromium kiosk on `localhost:8080` |
| Home network (phone, fridge, laptop) | `http://utilitypi.local:8080` (the server binds `0.0.0.0:8080`) |
| Anywhere, via Tailscale | `https://utilitypi.tailebfffc.ts.net` (Tailscale Serve, HTTPS; Funnel is **off**) |

---

## Services on utilitypi

| Unit | Runs | Notes |
|---|---|---|
| `dashboard.service` | `dashboard_server.py` | `EnvironmentFile=…/secrets.env`; no dependency on rtl_tcp |
| `eufy-stream.service` | `~/Eufy/start_eufy_stack.sh` | Drop-in `eufy-stream.service.d/secrets.conf` loads `secrets.env` (for `CAM_PASS`) |
| Docker `eufy-ws` | eufy-security-ws on :3000 | Started by the Eufy stack if not running |
| `go2rtc.service` | — | **Disabled** (superseded by the Eufy stack's go2rtc) |

Common commands:
```
sudo systemctl restart dashboard
journalctl -u dashboard -f
journalctl -u eufy-stream -f          # bridge shows [>] start / [<] stop per camera
curl -s localhost:8080/api/status | head -c 600
```

---

## Secrets

`secrets.env` (`chmod 600`, git-ignored), loaded by systemd:

```
FINNHUB_KEY=…
PIHOLE_PASSWORD=…
CAM_PASS=…
```

Nest login: `nest_issue_token.txt`, `nest_cookie.txt` (`chmod 600`, git-ignored via `nest_*.txt`).
Before committing, check that `git status --short` lists none of these.

---

## Network notes

- **utilitypi runs on Ethernet.** Its onboard WiFi (CYW43455) couldn't reliably rejoin the Xfinity gateway
  (auth timeouts, status 16); it works with other access points. A backup WiFi profile for `GVPSHA-IOT` (the
  AirPort Extreme, bridge mode, 10.0.0.11) exists.
- **Static addresses:**
  - Pi-hole: 10.0.0.9
  - AirPort Extreme: 10.0.0.11
  - Reolink driveway camera: 10.0.0.240
  - ESP32-CAM water meter reader: 10.0.0.241

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Electric/gas grey | `rtl_tcp` not listening on the Pi-hole (`systemctl status rtl_tcp` there), or rtlamr restarting (`journalctl -u dashboard`) |
| Water "camera offline" | ESP32-CAM live view open in a browser, or the device rebooting. Close the tab; check `curl http://10.0.0.241/json` |
| Driveway "camera unavailable" | Camera unplugged or moved, or go2rtc down (`journalctl -u eufy-stream`) |
| Camera tile "didn't respond" | Bridge not running, or eufy-ws needs re-login (`docker logs --tail 40 eufy-ws`) |
| Green bands on SoloCams | Someone changed the `_h264` lines back to `-tune zerolatency`, or turned `SOLOCAM_FORCE_H264` off |
| Climate "Google login expired" | Re-capture the cookie (see Climate) |
| Climate "Nest gateway: …" | Google gateway refused or unreachable; check internet; the stream retries with backoff |
| Pages missing after an update | The Pi's browser still has the old page: press F5 |
| Correlation line says "collecting" | Fewer than 10 trading days in `stock_history.csv` |

---

## Related tools (on the laptop)

- `plot_water.py`: plots an AI-on-the-edge daily log (`/fileserver/log/data/…csv`) into readings by outcome,
  per-drum values and sweep hand.
- `~/Arduino/libraries/GvpPasswords/GvpPasswords.h`: WiFi and API credentials for the ESP32 sketches, kept out
  of the sketch repos.
