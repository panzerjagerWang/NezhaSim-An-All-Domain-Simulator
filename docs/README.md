# NezhaSim — documentation website

A self-contained static site (no build step) for **NezhaSim**, the transmedium
air · surface · underwater · ground robotics simulator. Styled after the
[UUV Simulator](https://uuvsimulator.github.io/) documentation site.

## Pages

| File | Content |
|---|---|
| `index.html` | Landing page — animated `logo.gif` hero, overview, core innovations (PSC / LEAP / MT), domains, comparison table, quick start, citation. |
| `installation.html` | Build-from-source guide (Ubuntu 20.04 + ROS Noetic + Gazebo 11). |
| `tutorials.html` | Checked quick path: explicit water-exit command, Husky interface inspection, direct thruster control, and real force/phase telemetry names. |
| `architecture.html` | NSE/TR near-surface framing, PSC continuous-blend math + force figures, LEAP (equations + CFD & response-surface figures), MT, replaceable dynamic models, four-axis robustness. Figures sourced from the TASE revision and the task1–3 reviewer reports. |
| `robots.html` | Catalogue of robots, worlds, plugins, and the three validation scenarios. |

Assets live in `assets/` (`css/style.css`, `img/logo.gif`).

## Preview locally

```bash
cd docs
python3 -m http.server 8000
# open http://localhost:8000
```

## Publish on GitHub Pages

The repository's `.github/workflows/pages.yml` deploys this `docs/` directory on
every push to `main`. The navigation links already target the project repository.

Everything is plain HTML/CSS with a single CDN dependency (MathJax, only on the
Architecture page) — no Node, Jekyll or MkDocs toolchain required.
