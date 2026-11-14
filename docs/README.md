# NezhaSim — project website

A static site with no build step, deployed to GitHub Pages by
`.github/workflows/pages.yml` on every push to `main`.

## Pages

| File | Content |
|---|---|
| `index.html` | Project page: authors, highlight video, abstract, overview figure, PSC / LEAP / MT, validation results with the paper's numbers, robots, quick start, reproduction table, BibTeX. |
| `installation.html` | Build from source (Ubuntu 20.04, ROS Noetic, Gazebo 11). |
| `tutorials.html` | Take-off from the water, triple-domain Husky, thrusters, force telemetry, and reproduction of the paper. |
| `architecture.html` | Unified engine, PSC blending, LEAP models, MT, replaceable dynamics, limitations. |
| `robots.html` | Robots, worlds, plugins and the validation scenarios. |

Assets are in `assets/`: `css/style.css`, `js/site.js` (mobile menu and copy buttons),
`img/` (figures of the paper as WebP), `video/nezhasim_highlights.mp4` (37 s, muted) and
`fig_main.pdf`.

## Preview locally

```bash
cd docs
python3 -m http.server 8000
# open http://localhost:8000
```
