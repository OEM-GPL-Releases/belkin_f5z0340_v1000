#!/usr/bin/env bash

# Modified verison of getrev.sh.

# Doesn\'t attempt to probe metadata directory.  Simply issues svn,
# git, or hg command and deals with empty output.
# Doesn\'t prepend 'r' to front of revision number.

export LANG=C
export LC_ALL=C
[ -n "$TOPDIR" ] && cd $TOPDIR

try_version() {
	[ -f version ] || return 1
	REV="$(cat version)"
	[ -n "$REV" ]
}

try_svn() {
	REV="$(svn info 2>/dev/null | awk '/^Last Changed Rev:/ { print $4 }')"
	[ -n "$REV" ]
}

try_git() {
	REV="$(git log  2>/dev/null | grep -m 1 git-svn-id | awk '{ gsub(/.*@/, "", $2); print $2 }')"
	[ -n "$REV" ]
}

try_hg() {
	REV="$(hg log -r-1 --template '{desc}' 2>/dev/null | awk '{print $2}' | sed 's/\].*//')"
	[ -n "$REV" ]
}

try_version || try_svn || try_git || try_hg || REV="unknown"
echo "$REV"
