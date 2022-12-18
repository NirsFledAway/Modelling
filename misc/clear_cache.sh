# find ./ -name '*.slxc' | xargs rm
for f in $(find /tmp -name '*.slxc'); do rm "$f"; done
