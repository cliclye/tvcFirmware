import AppKit
import WebKit

/// Native window for EasyTVC Studio. Starts the local Python service (studio.py), then shows its
/// UI in a WKWebView. Nothing is served beyond 127.0.0.1; external links open in the browser.
final class StudioDelegate: NSObject, NSApplicationDelegate, WKNavigationDelegate, WKUIDelegate, WKDownloadDelegate {
    private var process: Process?
    private var studioURL: URL?
    private var output = Data()
    private var startupTimer: Timer?
    private var window: NSWindow?
    private var webView: WKWebView?

    func applicationDidFinishLaunching(_ notification: Notification) {
        buildMenu()
        guard let resource = Bundle.main.resourceURL else { fail("App resources are missing."); return }
        let paths = ["/opt/homebrew/bin/python3", "/usr/local/bin/python3", "/usr/bin/python3"]
        guard let python = paths.first(where: { FileManager.default.isExecutableFile(atPath: $0) }) else {
            fail("Python 3.9 or newer is needed. Install Python, then reopen Studio."); return
        }
        let child = Process()
        child.executableURL = URL(fileURLWithPath: python)
        // The firmware folder is the one this .app sits in.
        let project = Bundle.main.bundleURL.deletingLastPathComponent().path
        child.arguments = [resource.appendingPathComponent("studio.py").path, "--no-browser", "--project", project]
        child.currentDirectoryURL = resource
        let pipe = Pipe()
        child.standardOutput = pipe
        child.standardError = pipe
        pipe.fileHandleForReading.readabilityHandler = { [weak self] handle in
            let data = handle.availableData
            if data.isEmpty { handle.readabilityHandler = nil; return }
            DispatchQueue.main.async {
                guard let self = self else { return }
                self.output.append(data)
                if self.output.count > 16384 { self.output = self.output.suffix(16384) }
                let text = String(data: self.output, encoding: .utf8) ?? ""
                if self.studioURL == nil, let line = text.components(separatedBy: "\n").first(where: { $0.hasPrefix("EasyTVC Studio: http://127.0.0.1:") }),
                   let url = URL(string: String(line.dropFirst("EasyTVC Studio: ".count))) {
                    self.studioURL = url
                    self.startupTimer?.invalidate()
                    self.showWindow()
                }
            }
        }
        child.terminationHandler = { [weak self] process in
            DispatchQueue.main.async {
                if process.terminationStatus != 0 {
                    let diagnostic = String(data: self?.output ?? Data(), encoding: .utf8) ?? ""
                    self?.fail("The local service stopped.\n\(diagnostic)")
                } else { NSApp.terminate(nil) }
            }
        }
        process = child
        do { try child.run() } catch { fail(error.localizedDescription); return }
        startupTimer = Timer.scheduledTimer(withTimeInterval: 15, repeats: false) { [weak self] _ in
            self?.fail("The local service did not start within 15 seconds.")
        }
    }

    private func buildMenu() {
        let main = NSMenu()
        func submenu(_ title: String, _ items: [NSMenuItem]) {
            let holder = NSMenuItem()
            let menu = NSMenu(title: title)
            items.forEach { menu.addItem($0) }
            holder.submenu = menu
            main.addItem(holder)
        }
        func item(_ title: String, _ action: Selector?, _ key: String, target: AnyObject? = nil) -> NSMenuItem {
            let entry = NSMenuItem(title: title, action: action, keyEquivalent: key)
            entry.target = target
            return entry
        }
        submenu("EasyTVC Studio", [
            item("About EasyTVC Studio", #selector(NSApplication.orderFrontStandardAboutPanel(_:)), ""),
            .separator(),
            item("Hide EasyTVC Studio", #selector(NSApplication.hide(_:)), "h"),
            .separator(),
            item("Quit EasyTVC Studio", #selector(NSApplication.terminate(_:)), "q"),
        ])
        // Without an Edit menu, Cmd-C/V/A do nothing in the web view's text fields.
        submenu("Edit", [
            item("Undo", Selector(("undo:")), "z"), item("Redo", Selector(("redo:")), "Z"), .separator(),
            item("Cut", #selector(NSText.cut(_:)), "x"), item("Copy", #selector(NSText.copy(_:)), "c"),
            item("Paste", #selector(NSText.paste(_:)), "v"), item("Select All", #selector(NSText.selectAll(_:)), "a"),
        ])
        submenu("View", [
            item("Reload", #selector(reload), "r", target: self),
            item("Open in Browser", #selector(openInBrowser), "b", target: self),
        ])
        submenu("Window", [
            item("Minimize", #selector(NSWindow.performMiniaturize(_:)), "m"),
            item("Close", #selector(NSWindow.performClose(_:)), "w"),
        ])
        NSApp.mainMenu = main
    }

    private func showWindow() {
        guard let url = studioURL else { return }
        if let window = window { window.makeKeyAndOrderFront(nil); return }
        let configuration = WKWebViewConfiguration()
        configuration.websiteDataStore = .nonPersistent()
        let view = WKWebView(frame: .zero, configuration: configuration)
        view.navigationDelegate = self
        view.uiDelegate = self
        let frame = NSRect(x: 0, y: 0, width: 1400, height: 900)
        let window = NSWindow(contentRect: frame, styleMask: [.titled, .closable, .miniaturizable, .resizable],
                              backing: .buffered, defer: false)
        window.title = "EasyTVC Studio"
        window.minSize = NSSize(width: 820, height: 560)
        window.contentView = view
        window.setFrameAutosaveName("EasyTVCStudioMain")
        window.center()
        window.makeKeyAndOrderFront(nil)
        NSApp.activate(ignoringOtherApps: true)
        view.load(URLRequest(url: url))
        self.window = window
        self.webView = view
    }

    @objc func reload() { webView?.reload() }
    @objc func openInBrowser() { if let url = studioURL { NSWorkspace.shared.open(url) } }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }
    func applicationShouldHandleReopen(_ sender: NSApplication, hasVisibleWindows flag: Bool) -> Bool { showWindow(); return true }
    func applicationWillTerminate(_ notification: Notification) {
        startupTimer?.invalidate()
        process?.terminationHandler = nil
        if process?.isRunning == true { process?.terminate() }
    }

    // MARK: navigation: stay on the local service, send everything else to the browser
    private func isLocal(_ url: URL?) -> Bool {
        guard let url = url else { return false }
        if url.scheme == "blob" || url.scheme == "about" || url.scheme == "data" { return true }
        return url.host == "127.0.0.1" && url.port == studioURL?.port
    }

    func webView(_ webView: WKWebView, decidePolicyFor navigationAction: WKNavigationAction,
                 decisionHandler: @escaping (WKNavigationActionPolicy) -> Void) {
        if navigationAction.shouldPerformDownload { decisionHandler(.download); return }
        if !isLocal(navigationAction.request.url) {
            if let url = navigationAction.request.url { NSWorkspace.shared.open(url) }
            decisionHandler(.cancel); return
        }
        decisionHandler(.allow)
    }

    func webView(_ webView: WKWebView, navigationAction: WKNavigationAction, didBecome download: WKDownload) {
        download.delegate = self
    }

    func webView(_ webView: WKWebView, navigationResponse: WKNavigationResponse, didBecome download: WKDownload) {
        download.delegate = self
    }

    // MARK: downloads (CSV exports, saved profiles) go to ~/Downloads
    func download(_ download: WKDownload, decideDestinationUsing response: URLResponse, suggestedFilename: String,
                  completionHandler: @escaping (URL?) -> Void) {
        let folder = FileManager.default.urls(for: .downloadsDirectory, in: .userDomainMask)[0]
        let name = (suggestedFilename as NSString).deletingPathExtension
        let ext = (suggestedFilename as NSString).pathExtension
        var target = folder.appendingPathComponent(suggestedFilename)
        var n = 2
        while FileManager.default.fileExists(atPath: target.path) {
            target = folder.appendingPathComponent(ext.isEmpty ? "\(name) \(n)" : "\(name) \(n).\(ext)")
            n += 1
        }
        completionHandler(target)
    }

    func downloadDidFinish(_ download: WKDownload) {}

    func download(_ download: WKDownload, didFailWithError error: Error, resumeData: Data?) {
        alert("Could not save the file", error.localizedDescription)
    }

    // MARK: web UI hooks: file pickers, alerts, new windows
    func webView(_ webView: WKWebView, runOpenPanelWith parameters: WKOpenPanelParameters,
                 initiatedByFrame frame: WKFrameInfo, completionHandler: @escaping ([URL]?) -> Void) {
        let panel = NSOpenPanel()
        panel.allowsMultipleSelection = parameters.allowsMultipleSelection
        panel.canChooseDirectories = false
        panel.canChooseFiles = true
        guard let window = window else { completionHandler(nil); return }
        panel.beginSheetModal(for: window) { result in completionHandler(result == .OK ? panel.urls : nil) }
    }

    func webView(_ webView: WKWebView, runJavaScriptAlertPanelWithMessage message: String,
                 initiatedByFrame frame: WKFrameInfo, completionHandler: @escaping () -> Void) {
        alert("EasyTVC Studio", message)
        completionHandler()
    }

    func webView(_ webView: WKWebView, runJavaScriptConfirmPanelWithMessage message: String,
                 initiatedByFrame frame: WKFrameInfo, completionHandler: @escaping (Bool) -> Void) {
        let panel = NSAlert()
        panel.messageText = "EasyTVC Studio"
        panel.informativeText = message
        panel.addButton(withTitle: "OK")
        panel.addButton(withTitle: "Cancel")
        completionHandler(panel.runModal() == .alertFirstButtonReturn)
    }

    func webView(_ webView: WKWebView, createWebViewWith configuration: WKWebViewConfiguration,
                 for navigationAction: WKNavigationAction, windowFeatures: WKWindowFeatures) -> WKWebView? {
        if let url = navigationAction.request.url { NSWorkspace.shared.open(url) }
        return nil
    }

    private func alert(_ title: String, _ message: String) {
        let panel = NSAlert()
        panel.messageText = title
        panel.informativeText = message
        panel.runModal()
    }

    private func fail(_ message: String) {
        startupTimer?.invalidate()
        alert("EasyTVC Studio could not open", message)
        NSApp.terminate(nil)
    }
}

let app = NSApplication.shared
let delegate = StudioDelegate()
app.delegate = delegate
app.setActivationPolicy(.regular)
app.run()
