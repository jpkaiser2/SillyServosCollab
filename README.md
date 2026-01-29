# Silly Servos In-Progress Code

Welcome to our robotics team’s robot code repository!  
This repo is where we store our robot code, so everyone can collaborate, review changes, and keep a clean history.

This README explains how to:
1. Open this repo in **GitHub Codespaces**
2. Edit code safely (branches + commits)
3. Push changes back to GitHub
4. (Optional) open the project locally in Android Studio for building/deploying

---

## What you need
- A GitHub account
- Access to this repository (ask a captain/programmer if you can’t open it)
- A laptop is recommended, but Codespaces works on Chromebooks too

---

# 1) Opening the code in GitHub Codespaces

1. Go to the repository homepage on GitHub.
2. Click the green **Code** button.
3. Click the **Codespaces** tab.
4. Click **Create codespace on `main`** (or whatever the default branch is).

After a minute, a browser-based VS Code window will open. That’s your coding environment.

### If you don’t see “Codespaces”
- You may not be signed in
- Your account may not have access to Codespaces
- The repo may not allow Codespaces for your permission level  
Ask a team lead to help enable access.

---

# 2) Important: Don’t code directly on `main`

**Always create a branch** so we can review changes and avoid breaking the robot right before a match.

### Create a branch (recommended method)
1. Look at the bottom-left of VS Code for the branch name (probably `main`).
2. Click it → **Create new branch**
3. Name it something like:
   - `fix-intake-jacob`
   - `auto-red-left-tuning`
   - `teleop-driver-controls`

Now you’re working safely.

---

# 3) Editing code in Codespaces

In the left sidebar (Explorer), open files like:
- `TeamCode/src/main/java/...` (our custom code usually lives here)
- `FtcRobotController/...` (rarely edited)

Make your changes like normal VS Code:
- Use search: **Ctrl+F** (Cmd+F on Mac)
- Search across files: **Ctrl+Shift+F**
- Format code: **Shift+Alt+F** (may vary by OS)

---

# 4) Saving, committing, and pushing changes

Codespaces auto-saves your files, but you still need to **commit** and **push**.

### A) Commit your changes
1. Click the **Source Control** icon (looks like a branch / three dots connected).
2. You’ll see a list of changed files.
3. Write a clear commit message, like:
   - `Fix arm limit logic`
   - `Add driver slow mode toggle`
4. Click **Commit**.

### B) Push your changes to GitHub
After committing:
- Click **Sync Changes** / **Push** (button may vary)

Your branch now exists on GitHub with your changes.

---

# 5) Making a Pull Request (PR)

A Pull Request lets the programming lead review your changes before merging into `main`.

1. Go back to the repo on GitHub.
2. You’ll usually see a banner: **“Compare & pull request”** → click it  
   (or go to **Pull Requests** → **New Pull Request**)
3. Base branch: `main`  
   Compare branch: your branch
4. Add notes explaining:
   - What you changed
   - How to test it
5. Create the PR.

✅ After it’s approved, a lead will merge it into `main`.

---

# 6) Updating your Codespace with the latest `main`

If other people merged changes, update your branch:

