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

package httpapi

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/narsi97/3vn-robotics/vps/fleet/internal/store"
)

const token = "s3cret-token"

func newServer(t *testing.T, secret string) (http.Handler, *store.Store) {
	t.Helper()
	st := store.New()
	return New(st, secret, "/fleet-abc123").Routes(), st
}

func post(t *testing.T, h http.Handler, auth string, body any) *httptest.ResponseRecorder {
	t.Helper()
	buf := new(bytes.Buffer)
	switch v := body.(type) {
	case string:
		buf.WriteString(v)
	default:
		_ = json.NewEncoder(buf).Encode(v)
	}
	req := httptest.NewRequest(http.MethodPost, "/api/telemetry", buf)
	if auth != "" {
		req.Header.Set("Authorization", "Bearer "+auth)
	}
	rec := httptest.NewRecorder()
	h.ServeHTTP(rec, req)
	return rec
}

func TestHealthzIsAlwaysAvailable(t *testing.T) {
	h, _ := newServer(t, token)
	rec := httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, "/healthz", nil))
	if rec.Code != http.StatusOK {
		t.Fatalf("want 200, got %d", rec.Code)
	}
}

func TestAValidReportIsRecorded(t *testing.T) {
	h, st := newServer(t, token)
	rec := post(t, h, token, store.Report{RobotID: "robot-001", Ready: true})
	if rec.Code != http.StatusAccepted {
		t.Fatalf("want 202, got %d: %s", rec.Code, rec.Body)
	}
	if st.Count() != 1 {
		t.Fatal("the report was not stored")
	}
}

func TestReportsWithoutAValidTokenAreRefused(t *testing.T) {
	// This endpoint is reachable from the internet. Anything that can
	// write to the fleet view can lie about a robot's state.
	h, st := newServer(t, token)
	for name, auth := range map[string]string{
		"no token":    "",
		"wrong token": "not-the-token",
		"prefix only": token[:4],
	} {
		rec := post(t, h, auth, store.Report{RobotID: "robot-001"})
		if rec.Code != http.StatusUnauthorized {
			t.Errorf("%s: want 401, got %d", name, rec.Code)
		}
	}
	if st.Count() != 0 {
		t.Fatal("an unauthorised report reached the store")
	}
}

func TestIngestFailsClosedWhenNoSecretIsConfigured(t *testing.T) {
	// A missing token must not mean "accept everything". Failing closed
	// matters more than convenience for something internet-facing.
	h, st := newServer(t, "")
	rec := post(t, h, "anything", store.Report{RobotID: "robot-001"})
	if rec.Code != http.StatusServiceUnavailable {
		t.Fatalf("want 503, got %d", rec.Code)
	}
	if st.Count() != 0 {
		t.Fatal("a report was accepted with no secret configured")
	}
}

func TestAReportWithoutARobotIdIsRefused(t *testing.T) {
	// An unattributable report would appear in the fleet view as an
	// anonymous row, which is worse than not appearing at all.
	h, st := newServer(t, token)
	rec := post(t, h, token, store.Report{Ready: true})
	if rec.Code != http.StatusBadRequest {
		t.Fatalf("want 400, got %d", rec.Code)
	}
	if st.Count() != 0 {
		t.Fatal("an anonymous report was stored")
	}
}

func TestMalformedJsonIsRefusedWithoutPanicking(t *testing.T) {
	h, _ := newServer(t, token)
	rec := post(t, h, token, "{not json at all")
	if rec.Code != http.StatusBadRequest {
		t.Fatalf("want 400, got %d", rec.Code)
	}
}

func TestAnEnormousBodyIsRefusedRatherThanRead(t *testing.T) {
	// Unbounded reads on a 2 GB box shared with nine other products are
	// not something to leave open.
	h, _ := newServer(t, token)
	rec := post(t, h, token, `{"robot_id":"x","git_commit":"`+
		strings.Repeat("A", 64<<10)+`"}`)
	if rec.Code == http.StatusAccepted {
		t.Fatal("a body far over the cap was accepted")
	}
}

func TestGetOnTheIngestEndpointIsRefused(t *testing.T) {
	h, _ := newServer(t, token)
	rec := httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, "/api/telemetry", nil))
	if rec.Code != http.StatusMethodNotAllowed {
		t.Fatalf("want 405, got %d", rec.Code)
	}
}

func TestTheFleetPageRendersReportedRobots(t *testing.T) {
	h, _ := newServer(t, token)
	post(t, h, token, store.Report{
		RobotID: "robot-001", Ready: true, SoftwareVersion: "0.7.0",
	})

	rec := httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, "/", nil))
	if rec.Code != http.StatusOK {
		t.Fatalf("want 200, got %d", rec.Code)
	}
	body := rec.Body.String()
	for _, want := range []string{"robot-001", "0.7.0", "ready"} {
		if !strings.Contains(body, want) {
			t.Errorf("page does not mention %q", want)
		}
	}
}

func TestTheFleetPageIsNotIndexableOrCacheable(t *testing.T) {
	// It lives behind an unlisted path; keep it out of indexes even if
	// the path leaks, and never serve a cached view of a live system.
	h, _ := newServer(t, token)
	rec := httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, "/", nil))
	if got := rec.Header().Get("X-Robots-Tag"); !strings.Contains(got, "noindex") {
		t.Errorf("want noindex, got %q", got)
	}
	if got := rec.Header().Get("Cache-Control"); got != "no-store" {
		t.Errorf("want no-store, got %q", got)
	}
}

func TestTheEmptyFleetPageExplainsItself(t *testing.T) {
	h, _ := newServer(t, token)
	rec := httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, "/", nil))
	if !strings.Contains(rec.Body.String(), "THREEVN_FLEET_URL") {
		t.Error("an empty fleet should say how to enrol a robot")
	}
}

func TestAnUnknownPathIs404(t *testing.T) {
	h, _ := newServer(t, token)
	rec := httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, "/nope", nil))
	if rec.Code != http.StatusNotFound {
		t.Fatalf("want 404, got %d", rec.Code)
	}
}

func TestFleetJsonMatchesWhatWasReported(t *testing.T) {
	h, _ := newServer(t, token)
	post(t, h, token, store.Report{RobotID: "robot-001", JointMessages: 4017})

	rec := httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, "/api/fleet", nil))

	var got struct {
		Robots []store.Entry `json:"robots"`
	}
	if err := json.NewDecoder(rec.Body).Decode(&got); err != nil {
		t.Fatalf("response is not valid JSON: %v", err)
	}
	if len(got.Robots) != 1 || got.Robots[0].JointMessages != 4017 {
		t.Fatalf("unexpected payload: %+v", got.Robots)
	}
}
