.PHONY: all clean

all: micromini-tool micromini-tool.completions service-install
clean:
	rm -f micromini-tool micromini-tool.completions


micromini-tool.completions: micromini-tool
	echo complete -W \"`./micromini-tool --shortlist`\" ./micromini-tool > $@
	echo complete -W \"`./micromini-tool --shortlist`\" micromini-tool>> $@

micromini-tool: micromini-tool.c
	$(CC) -Wall -o $@ -g -Os $^

service-install: micromini-tool
	install *.service /etc/systemd/system
	systemctl daemon-reload
