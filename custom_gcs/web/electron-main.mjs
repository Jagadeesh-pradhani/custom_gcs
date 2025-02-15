import { app, BrowserWindow } from 'electron';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

async function createWindow() {
  const win = new BrowserWindow({
    width: 1280,
    height: 800,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false
    }
  });

  if (process.env.NODE_ENV === 'development') {
    await win.loadURL('http://localhost:5173');
    #win.webContents.openDevTools();
  } else {
    // Use proper path resolution for production
    const prodPath = path.join(__dirname, 'dist', 'index.html');
    console.log('Loading production file from:', prodPath); // Add logging
    await win.loadFile(prodPath);
  }
}

app.whenReady().then(createWindow);
