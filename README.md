# ESP32-S3-WROOM1-CAM-AI

PlatformIO project for ESP32-S3 (DFRobot FireBeetle 2 / N16R8) with camera and Edge Impulse deploy.

## Git workflow (Option 1 – fork for a similar project)

This repo is the **original** project. To start a similar project and still push improvements back here:

1. **Push this repo to GitHub** (one-time; already done for this repo)
   - Create a new repository on GitHub (e.g. `ESP32-S3-WROOM1-CAM-AI`). Do **not** add a README or .gitignore.
   - Then run (use your GitHub username in the URL):
   ```bash
   git remote add origin https://github.com/PTravnicek/ESP32-S3-WROOM1-CAM-AI.git
   git push -u origin main
   ```

2. **Fork for your new project**
   - On GitHub, open this repo and click **Fork**. Name the fork (e.g. `ESP32-S3-WROOM1-CAM-AI-NEW`).
   - Clone the fork and work there:
   ```bash
   git clone https://github.com/PTravnicek/ESP32-S3-WROOM1-CAM-AI-NEW.git
   cd ESP32-S3-WROOM1-CAM-AI-NEW
   ```

3. **Send improvements back to the original**
   - In your fork: commit changes, push to a branch.
   - On GitHub: open a **Pull Request** from your fork’s branch into this (original) repo.
   - Review and merge the PR here to bring the improvements back.

To pull updates from this original repo into your fork:

```bash
git remote add upstream https://github.com/PTravnicek/ESP32-S3-WROOM1-CAM-AI.git
git fetch upstream
git merge upstream/main
```
