#! /bin/bash

if [[ -e .git/shallow ]] && ! grep --silent '^SUBLEVEL = 0' Makefile; then
	# for stable branches ensure the history is deep enough to include the branch point as
	# otherwise git will mess up when looking up changes for Patchlist.changelog generation
	branchpoint=v$(make -s kernelversion | cut -d"." -f-2)
	echo "This is a shallow clone. Changing the dept of the tree to avoid oddities using"
	echo "$ git fetch --shallow-exclude=${branchpoint}"
	echo "This might flatten your history; press CTRL+C to abort."
	sleep 10
	git fetch --verbose --shallow-exclude="${branchpoint}" || :
fi
