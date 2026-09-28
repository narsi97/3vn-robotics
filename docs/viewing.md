# Three ways to look at the robot

There is no GPU in the container and no X server on macOS, so every view
is a remote view of some kind. They are not equivalent, and the browser
one — the default — is the slowest.

| | How to open | What crosses the boundary |
|---|---|---|
| **Browser (noVNC)** | `http://localhost:8106/vnc.html` | an encoded framebuffer, decoded in JavaScript |
| **Native VNC** | `make vnc` | the same framebuffer, decoded natively |
| **Foxglove** | `make foxglove` | **messages only** — your Mac draws the robot |

## 1. The browser: works everywhere, slowest

Nothing to install, and it is the right default for a first look. But
noVNC decodes every frame in JavaScript and repaints a canvas, on top of
RViz already rendering in software inside the container. If it feels
laggy, it is not your machine being slow — it is three layers of
software rasterisation stacked on each other.

## 2. Native VNC: same picture, less overhead

```bash
make vnc
```

Opens `vnc://127.0.0.1:5906`. Same desktop, same RViz, but the decode
happens in native code instead of a browser tab.

The port is published by compose and bound to **127.0.0.1 only**, which
is what makes it safe for x11vnc to run with `-nopw`. It is not
reachable from your network. Never rebind it to `0.0.0.0`.

macOS has a VNC client built in, so `make vnc` works with nothing
installed. It will ask for a password:

```
threevn
```

**There are two VNC servers in the container, and the password is why.**
x11vnc with `-nopw` offers only RFB security type 1, "None" — and native
clients refuse it. macOS Screen Sharing opens the TCP connection, never
finishes the handshake, and then reports *Connection failed*. Probing
the server directly showed it offering `[1]` whatever the client
announced, including Apple's own RFB 3.889:

```
RFB 003.889 -> security types [1]
RFB 003.008 -> security types [1]
```

So the container runs:

| port | auth | for |
|---|---|---|
| 5900, `-localhost` | none | websockify → the browser. Unreachable from the host. |
| 5901 → published as **5906** | VNC Authentication | native viewers |

The password **is not protecting anything** and is not a secret: 5906 is
bound to `127.0.0.1`, so anything that can reach it already runs as you
on this machine. It exists solely because clients refuse to complete a
handshake without one. Change it with `THREEVN_VNC_PASSWORD=... make down up`.

Tick "Remember this password" and you will not be asked again.

> **"Open Screen Sharing? A website wants to open this application."**
>
> You get this if you paste `vnc://127.0.0.1:5906` into a **browser**.
> It is the browser's ordinary hand-off prompt for a non-web URL scheme
> -- the same one `zoom://` or `slack://` produces -- and it is not a
> permission request. "Screen Sharing" is merely the NAME of the app
> macOS has registered for `vnc://`; nothing is asking for access to
> your screen, and your Mac's display is never captured or transmitted.
> The VNC server is inside the container and serves the virtual Xvfb
> display, which is the only screen it can see.
>
> `make vnc` avoids the prompt because `open` goes straight to
> LaunchServices with no web page in the middle.

**If it is still slow, shrink the desktop.** Every pixel is encoded and
shipped on every frame:

```bash
THREEVN_SCREEN=1280x720 make down up
```

## 3. Foxglove: stop shipping pixels

```bash
make foxglove          # prints what to do
make play RVIZ=0       # and run the sim without RViz
```

`foxglove_bridge` serves the ROS graph over a WebSocket on
`ws://127.0.0.1:8765`. Foxglove runs on your Mac, subscribes to the
topics a panel needs, and draws the robot **on your Mac's GPU** from
`/robot_description` and `/tf`. No framebuffer is transmitted at all, so
the view is as smooth as your Mac is — independent of how fast the
simulation is running.

This is the best view, with two caveats worth knowing before you install
anything:

- **Foxglove is a separate application** and recent versions want a
  (free) account. If that is not acceptable, native VNC is the fallback
  and costs nothing.
- **Version matters, and the failure is silent.** The bridge shipped
  here is `foxglove_bridge` 3.5.0, which speaks the `foxglove.sdk.v1`
  WebSocket subprotocol. An older Foxglove Studio asks for
  `foxglove.websocket.v1` and gets **HTTP 400 Bad Request** — which the
  app reports only as a failed connection. Verified by hand:

  ```
  foxglove.sdk.v1        -> 101 Switching Protocols
  foxglove.websocket.v1  -> 400 Bad Request
  ```

  If it will not connect, update Foxglove before debugging anything else.

### "Address already in use" on port 8765

```
[foxglove_bridge] Failed to bind port: Address already in use (os error 98)
```

A previous bridge is still holding the port. The rest of the stack comes
up perfectly, so the only symptom is that Foxglove will not connect and
the reason is buried in the launch output.

`make stop` kills it now -- it did not at first, which is exactly how
this was found. If you see it anyway:

```bash
make stop && make play
```

### Checking the bridge is up without installing anything

```bash
make play
```

then look for `Server listening on port 8765` in the output. From the
host, a plain TCP connect to `127.0.0.1:8765` succeeding is enough to
show the port is published.

## Which should you use

Start with the browser because it needs nothing. Move to `make vnc` when
it annoys you. Move to Foxglove if you are going to spend real time
watching the robot — it is the only one of the three that is not
fighting the CPU for frames.
