# Pre-release checklist

Run this before flipping the repository to Public and before every Release.
It takes about ten minutes; a leaked key takes much longer to undo.

## 1. Secrets and privacy

- [ ] `git log -p | rg -i "api[_-]?key|token|secret|password"` → no hits
- [ ] `git ls-files | rg "\.env$"` → only `.env.example`
- [ ] `git ls-files | rg -i "student|id_card|phone"` → nothing personal
- [ ] No internal IPs, lab machine names, or unpublished project names
- [ ] Screenshots show no usernames, paths, or notification popups

## 2. Large files and binaries

- [ ] `git ls-files -z | xargs -0 du -h | sort -h | tail -10` → largest < 10 MB
- [ ] `git ls-files | rg "\.(onnx|pt|ulg|bag|db3|mcap)$"` → no hits
- [ ] `du -sh .git` < 20 MB
- [ ] Weights and sample bags attached to the Release, not committed

## 3. Licensing

- [ ] `LICENSE` present and detected by GitHub (shows "MIT license")
- [ ] `NOTICE` lists PX4 (BSD-3), ROS 2 (Apache-2.0), Gazebo (Apache-2.0),
      Ultralytics (AGPL-3.0)
- [ ] README explains that the weights are AGPL-3.0 while the code is MIT
- [ ] Dataset license recorded in the README (if a dataset is used)

## 4. Honesty check

- [ ] No `显著`, `大幅`, `业界领先`, `state-of-the-art` in the README
- [ ] No claim of real flight if only SITL was run
- [ ] No claim of edge-device measurement if only x86 CPU was used
- [ ] Every number in the results table links to a CSV row
- [ ] Limitations section lists at least three real limitations

## 5. Reproducibility

- [ ] Fresh clone + `bash scripts/setup.sh` + `make e2e` works
- [ ] `requirements.lock.txt` committed and installable
- [ ] CI is green on `main`
- [ ] Release tag points at a commit where CI passed

## 6. Repository hygiene

- [ ] About description and Topics match the README (same keywords)
- [ ] Issues/Projects enabled, Wiki/Discussions disabled (unless used)
- [ ] `main` protected, no force pushes
- [ ] At least one merged PR with a green check

## Sign-off

| Date | Version | Checked by | Notes |
|---|---|---|---|
| | v0.1.0 | | |
