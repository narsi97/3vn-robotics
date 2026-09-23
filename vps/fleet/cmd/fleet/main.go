// Copyright 2026 3VN Systems
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Command fleet serves a read-only view of robots that report in.
//
// It runs on the shared 3VN VPS, which is 1 vCPU / 2 GB already carrying
// about ten containers. This is a single Go binary holding a handful of
// rows in memory, because the budget for anything else is not there.
//
// It accepts pushes and never initiates anything. Robots sit behind home
// NAT where inbound connections do not work, and more importantly a robot
// must never wait on this: losing the VPS has to degrade observability,
// never motion.
package main

import (
	"context"
	"errors"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/narsi97/3vn-robotics/vps/fleet/internal/httpapi"
	"github.com/narsi97/3vn-robotics/vps/fleet/internal/store"
)

func env(key, fallback string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return fallback
}

func main() {
	addr := ":" + env("PORT", "8080")
	secret := os.Getenv("FLEET_INGEST_TOKEN")
	basePath := env("FLEET_BASE_PATH", "")

	if secret == "" {
		// Loud, and it still starts: the view remains useful for whatever
		// has already reported, while ingest refuses rather than
		// accepting unauthenticated writes from the internet.
		log.Println("WARNING: FLEET_INGEST_TOKEN is unset; ingest will refuse all reports")
	}

	srv := &http.Server{
		Addr:    addr,
		Handler: httpapi.New(store.New(), secret, basePath).Routes(),
		// Bounded, because this is internet-facing on a small shared box.
		// Without these a handful of slow connections can hold sockets
		// open indefinitely.
		ReadHeaderTimeout: 5 * time.Second,
		ReadTimeout:       15 * time.Second,
		WriteTimeout:      15 * time.Second,
		IdleTimeout:       60 * time.Second,
	}

	go func() {
		log.Printf("fleet listening on %s (base path %q)", addr, basePath)
		if err := srv.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
			log.Fatalf("listen: %v", err)
		}
	}()

	stop := make(chan os.Signal, 1)
	signal.Notify(stop, os.Interrupt, syscall.SIGTERM)
	<-stop

	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	if err := srv.Shutdown(ctx); err != nil {
		log.Printf("shutdown: %v", err)
	}
	log.Println("stopped")
}
