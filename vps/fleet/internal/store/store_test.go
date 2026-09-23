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

package store

import (
	"sync"
	"testing"
	"time"
)

// clock lets a test move time without sleeping.
type clock struct{ t time.Time }

func (c *clock) now() time.Time      { return c.t }
func (c *clock) add(d time.Duration) { c.t = c.t.Add(d) }

func newTestStore() (*Store, *clock) {
	c := &clock{t: time.Date(2026, 1, 1, 12, 0, 0, 0, time.UTC)}
	return NewWithClock(c.now), c
}

func TestPutThenListReturnsTheReport(t *testing.T) {
	s, _ := newTestStore()
	s.Put(Report{RobotID: "robot-001", Ready: true, SoftwareVersion: "0.7.0"})

	rows := s.List()
	if len(rows) != 1 {
		t.Fatalf("want 1 row, got %d", len(rows))
	}
	if rows[0].RobotID != "robot-001" || !rows[0].Ready {
		t.Fatalf("unexpected row: %+v", rows[0])
	}
}

func TestASecondReportReplacesTheFirst(t *testing.T) {
	s, c := newTestStore()
	s.Put(Report{RobotID: "robot-001", Ready: true})
	c.add(5 * time.Second)
	s.Put(Report{RobotID: "robot-001", Ready: false})

	rows := s.List()
	if len(rows) != 1 {
		t.Fatalf("a robot must occupy one row, got %d", len(rows))
	}
	if rows[0].Ready {
		t.Fatal("the newer report should have replaced the older one")
	}
}

// The property that matters most. A robot that has stopped reporting must
// not keep showing as ready just because its last message said so -- that
// turns an outage into a silent one, which is worse than no dashboard.
func TestAStaleRobotIsNeverReportedReady(t *testing.T) {
	s, c := newTestStore()
	s.Put(Report{RobotID: "robot-001", Ready: true})

	if rows := s.List(); rows[0].Stale || !rows[0].Ready {
		t.Fatalf("should be fresh and ready immediately: %+v", rows[0])
	}

	c.add(StaleAfter + time.Second)

	rows := s.List()
	if !rows[0].Stale {
		t.Fatal("should be stale after the timeout")
	}
	if rows[0].Ready {
		t.Fatal("a stale robot must not be reported ready")
	}
}

func TestStalenessBoundaryIsInclusive(t *testing.T) {
	s, c := newTestStore()
	s.Put(Report{RobotID: "robot-001", Ready: true})
	c.add(StaleAfter)

	if rows := s.List(); rows[0].Stale {
		t.Fatal("exactly at the threshold should still count as fresh")
	}
}

func TestStalenessToleratesOneMissedReport(t *testing.T) {
	// Robots report every 10s. A single dropped request on a home
	// connection is not news, and an alarm that cries wolf gets ignored.
	s, c := newTestStore()
	s.Put(Report{RobotID: "robot-001", Ready: true})
	c.add(20 * time.Second)

	if rows := s.List(); rows[0].Stale {
		t.Fatal("two reporting intervals should not be treated as an outage")
	}
}

func TestFreshestRobotComesFirst(t *testing.T) {
	s, c := newTestStore()
	s.Put(Report{RobotID: "older"})
	c.add(10 * time.Second)
	s.Put(Report{RobotID: "newer"})

	rows := s.List()
	if rows[0].RobotID != "newer" {
		t.Fatalf("want newer first, got %q", rows[0].RobotID)
	}
}

func TestConcurrentWritesAndReadsDoNotRace(t *testing.T) {
	// Reports arrive on HTTP handler goroutines while the page renders on
	// another. Run with -race.
	s := New()
	var wg sync.WaitGroup
	for i := 0; i < 50; i++ {
		wg.Add(2)
		go func(n int) { defer wg.Done(); s.Put(Report{RobotID: "robot-001"}) }(i)
		go func() { defer wg.Done(); _ = s.List() }()
	}
	wg.Wait()
	if s.Count() != 1 {
		t.Fatalf("want 1 robot, got %d", s.Count())
	}
}
