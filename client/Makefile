.PHONY: all clean

all: micromini-tool micromini-tool.completions
clean:
	rm -f micromini-tool micromini-tool.completions


micromini-tool.completions: micromini-tool
	echo complete -W \"`./micromini-tool --shortlist`\" ./micromini-tool > $@
	echo complete -W \"`./micromini-tool --shortlist`\" micromini-tool>> $@

micromini-tool: micromini-tool.c
	$(CC) -o $@ -g -Os $^


