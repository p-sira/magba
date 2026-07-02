PANDOC = pandoc
SRC = paper.md
OUT = paper.pdf
BIB = paper.bib

all: $(OUT)

$(OUT): $(SRC) $(BIB)
	$(PANDOC) $(SRC) \
		-o $(OUT) \
		--citeproc \
		--bibliography=$(BIB) \
		--pdf-engine=xelatex \
		-V papersize:a4 \
		-V geometry:margin=1in \
		-V geometry:top=0.75in \
		-M link-citations=true

clean:
	rm -f $(OUT)

.PHONY: all clean