import { app as e, BrowserWindow as o } from "electron";
import t from "node:path";
function i() {
  const n = new o({
    width: 1280,
    height: 800,
    webPreferences: {
      nodeIntegration: !1,
      contextIsolation: !0
    }
  });
  process.env.VITE_DEV_SERVER_URL ? n.loadURL(process.env.VITE_DEV_SERVER_URL) : n.loadFile(t.join(__dirname, "../dist/index.html"));
}
e.whenReady().then(i);
e.on("window-all-closed", () => {
  process.platform !== "darwin" && e.quit();
});
e.on("activate", () => {
  o.getAllWindows().length === 0 && i();
});
