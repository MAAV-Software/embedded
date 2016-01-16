PDFS := kalman.pdf ccs_build.pdf meetings/notes_11-24.pdf 

TEXCC := pdflatex
GLOCC := makeglossaries
BIBCC := bibtex
BAD_EXTS := log aux toc xdy glo out acn acr alg glg gls ist bbl blg nav snm

all: $(PDFS)

clean :
	rm -f *.pdf $(patsubst %, *.%, $(BAD_EXTS))

%.pdf : %.tex
	$(TEXCC) $<
	$(BIBCC) $(patsubst %.tex, %, $<) || exit 0
	$(TEXCC) $<
	$(TEXCC) $<
	rm -f $(patsubst %, *.%, $(BAD_EXTS))

%.pdf : %.svg
	cd $(dir $<) && convert $(notdir $<) $(notdir $@)

%.png : %.xcf
	cd $(dir $<) && convert $(notdir $<) $(notdir $@)

.PHONY : all clean
