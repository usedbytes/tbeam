// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

package main

import (
	"encoding/binary"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"time"

	"github.com/twpayne/go-gpx"
)

type Semicircles int32
func (s Semicircles) Degrees() float64 {
	return float64(s) * 180 / math.Pow(2, 31)
}

type Timestamp uint32
func (t Timestamp) Time() time.Time {
	return time.Date(2020, time.May, 0, 0, 0, 0, 0, time.UTC).Add(time.Duration(t) * time.Second)
}

type Millimetres uint32
func (m Millimetres) Metres() float64 {
	return float64(m) / 1000
}

type Record struct {
	Timestamp Timestamp
	Longitude Semicircles
	Latitude  Semicircles
	Accuracy  Millimetres
}

func (r Record) String() string {
	los := 'E'
	long := r.Longitude.Degrees()
	if long < 0 {
		los = 'W'
	}

	las := 'N'
	lat := r.Latitude.Degrees()
	if lat < 0 {
		las = 'S'
	}
	return fmt.Sprintf("%v: %2.4f %c, %2.4f %c, %2.4f m", r.Timestamp.Time(), long, los, lat, las, r.Accuracy.Metres())
}

func run() error {
	if len(os.Args) != 2 {
		return fmt.Errorf("usage: %s INPUT_FILE", os.Args[0]);
	}

	inName := os.Args[1]

	f, err := os.Open(inName)
	if err != nil {
		return err
	}

	fi, err := f.Stat()
	if err != nil {
		return err
	}

	numRecords := fi.Size() / 16

	var records []Record = make([]Record, numRecords)
	err = binary.Read(f, binary.LittleEndian, records)
	if err != nil {
		return err
	}

	g := &gpx.GPX{
		Trk: make([]*gpx.TrkType, 0, 1),
	}

	trk := &gpx.TrkType{
		TrkSeg: make([]*gpx.TrkSegType, 1),
	}
	g.Trk = append(g.Trk, trk)

	seg := &gpx.TrkSegType{
		TrkPt: make([]*gpx.WptType, 0, numRecords),
	}
	trk.TrkSeg[0] = seg

	for _, r := range records {
		fmt.Println(r)
	}

	for _, rec := range records {
		pt := &gpx.WptType{
			Lat: rec.Latitude.Degrees(),
			Lon: rec.Longitude.Degrees(),
			Time: rec.Timestamp.Time(),
		}

		seg.TrkPt = append(seg.TrkPt, pt)
	}

	outName := filepath.Base(inName)
	outName = outName[0:len(outName) - len(filepath.Ext(outName))] + ".gpx"

	outFile, err := os.Create(outName)
	if err != nil {
		return err
	}
	defer outFile.Close()

	err = g.Write(outFile)
	if err != nil {
		return err
	}

	return nil
}

func main() {
	err := run()
	if err != nil {
		fmt.Println(err)
		os.Exit(1)
	}
}
