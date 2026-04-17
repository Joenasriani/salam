# Deploying RoboNet Editor to Vercel

## Quick Deploy Steps

### 1. Push to GitHub
```bash
cd /workspace/robosim_studio
git add .
git commit -m "Add Vercel-ready frontend"
git push origin main
```

### 2. Connect to Vercel
1. Go to [vercel.com](https://vercel.com)
2. Click "New Project"
3. Import your GitHub repository
4. Select the `frontend` folder as the **Root Directory**
5. Use these settings:

| Setting | Value |
|---------|-------|
| **Framework Preset** | `Other` |
| **Build Command** | `echo 'No build required'` |
| **Output Directory** | `.` |
| **Install Command** | `echo 'No install required'` |
| **Development Command** | `npx serve .` |

6. Click **Deploy**

### 3. Configure Backend URL (Optional)
If you deploy the Python backend separately:
1. In Vercel Dashboard → Settings → Environment Variables
2. Add: `REACT_APP_WS_URL = wss://your-backend-url.com`
3. Redeploy

## Important Notes

⚠️ **Frontend Only**: This deploys ONLY the visual editor. The PyBullet physics simulation backend CANNOT run on Vercel.

✅ **What Works on Vercel**:
- Visual node editor
- Drag-and-drop interface
- Parameter editing
- JSON export/download
- Mock simulation mode

❌ **What Doesn't Work on Vercel**:
- Real PyBullet physics simulation
- WebSocket connections (unless backend deployed elsewhere)
- Hardware control

## Backend Deployment (Separate)

Deploy the Python backend to one of these platforms:

### Railway (Recommended)
```bash
# Create railway.toml in root
pip install railway
railway login
railway init
railway up
```

### Render
1. Create new Web Service
2. Connect GitHub repo
3. Build Command: `pip install -r requirements.txt`
4. Start Command: `python server/websocket_server.py`

### Fly.io
```bash
flyctl launch
flyctl deploy
```

## Architecture Diagram

```
┌─────────────────┐         WebSocket          ┌──────────────────┐
│   Vercel CDN    │ ◄───────────────────────► │  Railway/Render  │
│  (Frontend UI)  │      ws://backend:8765    │ (PyBullet + RBP) │
│                 │                            │                  │
│ - React Flow    │                            │ - Physics Engine │
│ - Node Editor   │                            │ - Scenario Runner│
│ - JSON Export   │                            │ - Collision Check│
└─────────────────┘                            └──────────────────┘
```

## Testing Locally

```bash
# Terminal 1: Start backend
cd robosim_studio
python server/websocket_server.py

# Terminal 2: Serve frontend
cd robosim_studio/frontend
npx serve .

# Open http://localhost:3000
```

## Production URLs

After deployment:
- Frontend: `https://your-app.vercel.app`
- Backend: `wss://your-app.railway.app`

Update `REACT_APP_WS_URL` in Vercel to point to your backend.
