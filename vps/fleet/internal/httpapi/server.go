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

// Package httpapi serves the fleet view and accepts robot reports.
package httpapi

import (
	"crypto/subtle"
	"encoding/json"
	"html/template"
	"io"
	"net/http"
	"strings"
	"time"

	"github.com/narsi97/3vn-robotics/vps/fleet/internal/store"
)

// maxBody caps a report. A fleet report is a few hundred bytes; anything
// larger is a mistake or an attack, and an unbounded read on a 2 GB box
// shared with nine other products is not something to leave open.
const maxBody = 32 << 10

// Server is the HTTP surface.
type Server struct {
	store  *store.Store
	secret string
	// basePath is the unlisted prefix the shared Caddy strips. Routes are
	// registered without it; this is only used to build links in the page.
	basePath string
	tmpl     *template.Template
}

// New builds a Server. An empty secret disables ingest entirely rather
// than accepting unauthenticated writes — failing closed matters more
// than convenience for something reachable from the internet.
func New(st *store.Store, secret, basePath string) *Server {
	return &Server{
		store:    st,
		secret:   secret,
		basePath: strings.TrimSuffix(basePath, "/"),
		tmpl:     template.Must(template.New("fleet").Parse(fleetHTML)),
	}
}

// Routes returns the mux.
func (s *Server) Routes() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("/healthz", s.healthz)
	mux.HandleFunc("/api/telemetry", s.ingest)
	mux.HandleFunc("/api/fleet", s.fleetJSON)
	mux.HandleFunc("/", s.fleetPage)
	return mux
}

func (s *Server) healthz(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, map[string]any{
		"status": "ok",
		"robots": s.store.Count(),
	})
}

// ingest accepts one robot's report.
func (s *Server) ingest(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	if s.secret == "" {
		// No secret configured means no authenticated writes are
		// possible, so refuse rather than accept anything.
		writeJSON(w, http.StatusServiceUnavailable,
			map[string]string{"error": "ingest is not configured"})
		return
	}

	// Constant-time compare: a token checked with == leaks its prefix
	// through timing, and this endpoint is public.
	given := strings.TrimPrefix(r.Header.Get("Authorization"), "Bearer ")
	if subtle.ConstantTimeCompare([]byte(given), []byte(s.secret)) != 1 {
		writeJSON(w, http.StatusUnauthorized, map[string]string{"error": "bad token"})
		return
	}

	var report store.Report
	if err := json.NewDecoder(io.LimitReader(r.Body, maxBody)).Decode(&report); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "malformed report"})
		return
	}
	if strings.TrimSpace(report.RobotID) == "" {
		// A report nobody can attribute is worse than no report: it would
		// appear in the fleet view as an anonymous row.
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "robot_id is required"})
		return
	}

	s.store.Put(report)
	writeJSON(w, http.StatusAccepted, map[string]string{"status": "recorded"})
}

func (s *Server) fleetJSON(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, map[string]any{
		"robots":       s.store.List(),
		"generated_at": time.Now().UTC().Format(time.RFC3339),
	})
}

func (s *Server) fleetPage(w http.ResponseWriter, r *http.Request) {
	if r.URL.Path != "/" {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "not found"})
		return
	}
	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	// A cached fleet view is actively misleading about a live system.
	w.Header().Set("Cache-Control", "no-store")
	// This page is served under an unlisted path; keep it out of indexes
	// even if the path leaks.
	w.Header().Set("X-Robots-Tag", "noindex, nofollow")
	_ = s.tmpl.Execute(w, map[string]any{
		"Robots":   s.store.List(),
		"BasePath": s.basePath,
	})
}

func writeJSON(w http.ResponseWriter, code int, body any) {
	w.Header().Set("Content-Type", "application/json")
	w.Header().Set("Cache-Control", "no-store")
	w.WriteHeader(code)
	_ = json.NewEncoder(w).Encode(body)
}
