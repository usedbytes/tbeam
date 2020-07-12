package main

import (
	"fmt"
	"io/ioutil"
	"net/http"
	"net/http/httputil"
	"os"
	"path/filepath"
)

func run() error {

	fmt.Println("Listening...")

	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {

		dump, err := httputil.DumpRequest(r, false)
		if err != nil {
			fmt.Println("request dump failed")
			http.Error(w, fmt.Sprint(err), http.StatusInternalServerError)
			return
		}
		fmt.Printf("%q\n", dump)

		err = r.ParseForm()
		if err != nil {
			fmt.Println("failed to parse form")
			http.Error(w, fmt.Sprint(err), http.StatusInternalServerError)
		}

		b, err := ioutil.ReadAll(r.Body)
		if err != nil {
			fmt.Println("failed to read body")
			http.Error(w, fmt.Sprint(err), http.StatusInternalServerError)
		}

		v := r.Form.Get("filename")
		if v != "" {
			fname := filepath.Clean(filepath.Join(".", v))

			fmt.Println("Writing to", fname)

			f, err := os.Create(fname)
			if err != nil {
				fmt.Println(err)
				http.Error(w, fmt.Sprint(err), http.StatusInternalServerError)
				return
			}
			defer f.Close()

			_, err = f.Write(b)
			if err != nil {
				fmt.Println(err)
				http.Error(w, fmt.Sprint(err), http.StatusInternalServerError)
				return
			}
		}
	})
	return http.ListenAndServe(":8080", nil)
}

func main() {
	err := run()
	if err != nil {
		fmt.Println(err)
		os.Exit(1)
	}
}
