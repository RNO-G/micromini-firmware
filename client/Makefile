.PHONY: all clean

all: micromini-tool micromini-tool.completions

clean:
	rm -f micromini-tool micromini-tool.completions


micromini-tool.completions: micromini-tool
	echo complete -W \"`./micromini-tool --shortlist`\" ./micromini-tool > $@
	echo complete -W \"`./micromini-tool --shortlist`\" micromini-tool>> $@

micromini-tool: micromini-tool.c
	$(CC) -Wall -o $@ -g -Os $^

#this doesn't need sudo unless /rno-g/bin has wrong permissions
install: micromini-tool
	install micromini-tool /rno-g/bin
	install log_power.py /rno-g/bin

# these need sudo
completion-install: micromini-tool.completions
	install micromini-tool.completions /etc/bash_completion.d/micromini-tool

service-install: micromini-tool
	sudo install *.service /etc/systemd/system
	sudo systemctl daemon-reload
