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

// Package store keeps the latest report from each robot.
//
// In memory, on purpose. This holds a handful of rows describing what
// robots said moments ago; it is a view, not a record. Losing it on
// restart costs one reporting interval, and the alternative — a Postgres
// database and its migrations, backups and connection pool — buys
// durability for data whose value expires in seconds.
//
// Phase 9 adds real telemetry over OpenTelemetry, where history belongs.
// This exists so a robot can be seen at all before that lands.
package store

import (
	"sort"
	"sync"
	"time"
)

// StaleAfter is how long a robot may go unheard before it is shown as
// stale. Robots report every 10s, so this tolerates two missed reports
// before drawing conclusions — a single dropped request on a home
// connection is not news.
const StaleAfter = 35 * time.Second

// Report is what a robot sends. Deliberately a subset of what the
// robot's own dashboard knows: joint angles at 50 Hz have no business
// crossing the internet, and a fleet view needs health, not kinematics.
type Report struct {
	RobotID         string   `json:"robot_id"`
	SoftwareVersion string   `json:"software_version"`
	FirmwareVersion string   `json:"firmware_version"`
	GitCommit       string   `json:"git_commit"`
	RobotModel      string   `json:"robot_model"`
	HardwareTarget  string   `json:"hardware_target"`
	Ready           bool     `json:"ready"`
	UptimeSeconds   float64  `json:"uptime_seconds"`
	JointMessages   int64    `json:"joint_messages"`
	Reasons         []string `json:"reasons,omitempty"`
}

// Entry is a Report plus what the server knows about it.
type Entry struct {
	Report
	ReceivedAt time.Time `json:"received_at"`
	// Stale means nothing has arrived recently. A robot that stopped
	// reporting is NOT reported as healthy just because its last message
	// said so — that would turn an outage into a silent one.
	Stale bool `json:"stale"`
	// AgeSeconds since the last report.
	AgeSeconds float64 `json:"age_seconds"`
}

// Store holds the latest entry per robot.
type Store struct {
	mu   sync.RWMutex
	now  func() time.Time
	rows map[string]Entry
}

// New returns an empty Store.
func New() *Store { return NewWithClock(time.Now) }

// NewWithClock lets tests drive time by hand, so staleness can be
// exercised without sleeping.
func NewWithClock(now func() time.Time) *Store {
	return &Store{now: now, rows: make(map[string]Entry)}
}

// Put records a report, replacing any previous one from that robot.
func (s *Store) Put(r Report) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.rows[r.RobotID] = Entry{Report: r, ReceivedAt: s.now()}
}

// List returns every robot, freshest first, with staleness computed at
// read time rather than stored — a row does not become stale by being
// written to, it becomes stale by the passage of time.
func (s *Store) List() []Entry {
	s.mu.RLock()
	defer s.mu.RUnlock()

	now := s.now()
	out := make([]Entry, 0, len(s.rows))
	for _, e := range s.rows {
		age := now.Sub(e.ReceivedAt)
		e.AgeSeconds = age.Seconds()
		e.Stale = age > StaleAfter
		// A stale row cannot claim to be ready. Whatever it last said is
		// no longer evidence of anything.
		if e.Stale {
			e.Ready = false
		}
		out = append(out, e)
	}
	sort.Slice(out, func(i, j int) bool {
		if out[i].AgeSeconds != out[j].AgeSeconds {
			return out[i].AgeSeconds < out[j].AgeSeconds
		}
		return out[i].RobotID < out[j].RobotID
	})
	return out
}

// Count returns how many robots have ever reported.
func (s *Store) Count() int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return len(s.rows)
}
