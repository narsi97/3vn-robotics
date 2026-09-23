# 3VN fleet view.
#
# House style, matching the other 3VN Go backends: multi-stage
# golang-alpine -> alpine, CGO_ENABLED=0, non-root numeric UID.
#
# It runs on the shared VPS, which is 1 vCPU / 2 GB already carrying
# about ten containers plus Umami. Every megabyte of RSS here is a
# megabyte the other products do not get, which is why this is a static
# binary holding a handful of rows in memory rather than anything with a
# database behind it.
FROM golang:1.23-alpine AS build
WORKDIR /src

# Dependencies first, so a source-only change does not refetch them.
# (This module has no external dependencies today; the layer is here so
# that stays true when it gains one.)
COPY go.mod ./
RUN go mod download

COPY . .

# CGO_ENABLED=0 for a fully static binary, so the final stage needs no
# libc and nothing can drift between build and run.
# -trimpath and -ldflags strip build paths and debug info: smaller, and
# no absolute paths from a build machine end up in a shipped artifact.
RUN CGO_ENABLED=0 GOOS=linux go build \
      -trimpath -ldflags="-s -w" \
      -o /out/fleet ./cmd/fleet

FROM alpine:3.20
# wget is used by the healthcheck only; alpine ships busybox wget.
RUN apk add --no-cache ca-certificates

# Non-root numeric UID, no home. This binary writes nothing to disk.
RUN adduser -D -H -u 10001 app
COPY --from=build /out/fleet /usr/local/bin/fleet

USER app
EXPOSE 8080

# 127.0.0.1, never localhost -- house rule. The binary binds IPv4 and a
# container's /etc/hosts resolves localhost to ::1 first, so a localhost
# healthcheck fails against a perfectly healthy service.
HEALTHCHECK --interval=10s --timeout=3s --retries=6 \
  CMD wget -qO- http://127.0.0.1:8080/healthz >/dev/null || exit 1

ENTRYPOINT ["/usr/local/bin/fleet"]
