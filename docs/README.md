# NezhaSim — documentation website

A self-contained static site (no build step) for **NezhaSim**, the transmedium
air · surface · underwater · ground robotics simulator. Styled after the
[UUV Simulator](https://uuvsimulator.github.io/) documentation site.

## Pages

| File | Content |
|---|---|
| `index.html` | Landing page — animated `logo.gif` hero, overview, core innovations (PSC / LEAP / MT), domains, comparison table, quick start, citation. |
| `installation.html` | Build-from-source guide (Ubuntu 20.04 + ROS Noetic + Gazebo 11). |
| `tutorials.html` | Five step-by-step tutorials: water take-off, triple-domain Husky mission, underwater control, reading the MT force streams, gantry force ID. |
| `architecture.html` | NSE/TR near-surface framing, PSC continuous-blend math + force figures, LEAP (equations + CFD & response-surface figures), MT, replaceable dynamic models, four-axis robustness. Figures sourced from the TASE revision and the task1–3 reviewer reports. |
| `robots.html` | Catalogue of robots, worlds, plugins, and the three validation scenarios. |

Assets live in `assets/` (`css/style.css`, `img/logo.gif`).

## Preview locally

```bash
cd website
python3 -m http.server 8000
# open http://localhost:8000
```

## Publish on GitHub Pages

1. Push this `website/` folder to your repo.
2. In **Settings → Pages**, set the source to the branch and the `/website`
   folder (or rename it to `docs/` and select `/docs`).
3. Update the `GitHub ↗` links in the nav (currently `https://github.com/`) to
   your repository URL.

Everything is plain HTML/CSS with a single CDN dependency (MathJax, only on the
Architecture page) — no Node, Jekyll or MkDocs toolchain required.
