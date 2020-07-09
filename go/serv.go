package main

import (
	"fmt"
	"io/ioutil"
	"net/http"
	"os"
)

func run() error {
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		err := r.ParseForm()
		if err != nil {
			fmt.Println("failed to parse form")
		}

		b, err := ioutil.ReadAll(r.Body)
		if err != nil {
			fmt.Println("failed to read body")
		}

		s := string(b)
		fmt.Printf("%v\n", s)
		fmt.Printf("%+v\n", r.Form)
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
