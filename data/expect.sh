#!/usr/bin/env bash
#
# Compare a `key=value` line the replay harness prints against expectations.
#
#   source data/expect.sh
#   compare_pairs "$score" "pos_h<=101.112 in3s>=0.5294" mission
#
#   data/expect.sh --self-test       fixtures with known verdicts
#
# Two files hold expectations against two lines the harness prints: `data/manifest.txt`
# against `summary`, read by `data/fetch.sh --check`, and `data/scenarios.txt` against
# `score`, read by `data/bench.sh`. They differ in what produces the input -- a converted
# ULog, a generated scenario -- and agree on everything after it, so the comparison lives
# here once and the pair syntax is one language a reader learns once.
#
# Three forms, because a corpus expectation and a benchmark ceiling want different things:
#
#   key=value    exact, string. What the corpus pins: `status=Healthy`, `rate=250`.
#   key<=value   numeric ceiling. An error the filter must not exceed.
#   key>=value   numeric floor. A goodness key -- `in3s`, `scored`.
#
# A key named here and absent from the line is a failure, not a skip: renaming a `score` or
# `summary` key would otherwise silently stop checking every expectation that named it.
#
# So is a value that is not a number where one is required. `nees_pos=none` is what the
# harness prints when a covariance block does not invert, and awk reads it as zero, which
# passes every ceiling. That is the one way a gate can report success for a filter it never
# measured.

# Read `key=` out of a line of `key=value` pairs. Fails if the key is absent. The match is
# on the whole key, so a ceiling on `pos_h` does not read `pos_h_max`.
#
# Splitting the line needs the unquoted expansion, which also invites pathname expansion: a
# value containing `*`, `?` or `[...]` would be replaced by whatever matches it in whatever
# directory the caller happens to be in, and the key would then misread or go missing. No
# `score` or `summary` key can hold one today; `set -f` is what keeps that from being a
# property this function depends on.
pair_value() {
    local line=$1 key=$2 word
    local restore_glob=0
    case $- in *f*) ;; *) restore_glob=1; set -f ;; esac
    for word in $line; do
        case "$word" in
            "$key"=*)
                printf '%s' "${word#*=}"
                [ "$restore_glob" = 1 ] && set +f
                return 0
                ;;
        esac
    done
    [ "$restore_glob" = 1 ] && set +f
    return 1
}

# 0 pass, 1 fail, 2 either side is not a number. Comparison is numeric rather than lexical:
# a ceiling of 10.0 is not breached by 9.9, which is what string ordering would conclude.
numeric_ok() {
    awk -v a="$1" -v op="$2" -v b="$3" 'BEGIN {
        number = "^-?[0-9]+([.][0-9]+)?$"
        if (a !~ number || b !~ number) exit 2
        exit !(op == "<=" ? a + 0 <= b + 0 : a + 0 >= b + 0)
    }'
}

# compare_pairs <line> <expectations> [label]
#
# Reports every failure rather than the first, so one run names everything that moved.
compare_pairs() {
    local line=$1 expect=$2 label=${3:-} rc=0 pair key op want got
    # Both loops here split on an unquoted expansion -- this one over the expectations, the
    # one in `pair_value` over the line -- so both are open to pathname expansion. Disabled
    # for the whole function, which covers `pair_value` too, since the option is global.
    local restore_glob=0
    case $- in *f*) ;; *) restore_glob=1; set -f ;; esac
    for pair in $expect; do
        case "$pair" in
            *'<='*) key=${pair%%<=*} op='<=' want=${pair##*<=} ;;
            *'>='*) key=${pair%%>=*} op='>=' want=${pair##*>=} ;;
            *=*)    key=${pair%%=*}  op='='  want=${pair#*=} ;;
            *) echo "  MALFORMED  ${label:+$label: }$pair has no =, <= or >=" >&2; rc=1; continue ;;
        esac
        if ! got=$(pair_value "$line" "$key"); then
            echo "  MISSING    ${label:+$label: }no \`$key=\` on the line, wanted $pair" >&2
            rc=1
            continue
        fi
        if [ "$op" = '=' ]; then
            [ "$got" = "$want" ] || { echo "  MISMATCH   ${label:+$label: }$key=$got, wanted $pair" >&2; rc=1; }
            continue
        fi
        numeric_ok "$got" "$op" "$want"
        case $? in
            0) ;;
            1) echo "  BREACH     ${label:+$label: }$key=$got, wanted $pair" >&2; rc=1 ;;
            *) echo "  NOT A NUMBER ${label:+$label: }$key=$got cannot be compared against $pair" >&2; rc=1 ;;
        esac
    done
    [ "$restore_glob" = 1 ] && set +f
    return $rc
}

# The expectations in data/manifest.txt and data/scenarios.txt were produced by the harness
# they are meant to guard, so a comparator that waves something through turns a miscount into
# a pinned number and then into the baseline every later change is measured against -- the
# reason examples/replay.rs carries its own tests. These fixtures are the same argument one
# level down: every line is a literal, and every verdict beside it is one somebody can check
# by reading.
self_test() {
    local passed=0 failed=0

    # t <expected rc> <name> <line> <expectations>
    t() {
        local want=$1 name=$2 line=$3 expect=$4 got=0
        compare_pairs "$line" "$expect" 2>/dev/null || got=$?
        if [ "$got" = "$want" ]; then
            passed=$((passed + 1))
        else
            failed=$((failed + 1))
            echo "  FAIL  $name: rc=$got, wanted $want" >&2
        fi
    }

    local score='score pos_h=101.112 pos_v=14.025 in3s=0.5294 nees_att=none false_valid=179874 scored=36600'
    local summary='summary rate=250 align=static heading=valid status=Healthy'

    t 0 'exact match'                "$summary" 'status=Healthy'
    t 1 'exact mismatch'             "$summary" 'status=Degraded'
    t 0 'several pairs, all held'    "$summary" 'rate=250 align=static heading=valid'
    t 1 'one pair of several broken' "$summary" 'rate=250 align=coarse heading=valid'
    t 0 'no expectations'            "$summary" ''

    t 0 'ceiling with room'          "$score" 'pos_h<=200'
    t 0 'ceiling exactly met'        "$score" 'pos_h<=101.112'
    t 1 'ceiling breached'           "$score" 'pos_h<=101.111'
    t 0 'floor exactly met'          "$score" 'scored>=36600'
    t 1 'floor breached'             "$score" 'scored>=36601'
    t 0 'floor with room'            "$score" 'in3s>=0.5'

    # Lexical ordering says 9.9 > 10.0, and every ceiling in data/scenarios.txt is a decimal.
    t 0 'decimals compare as numbers' 'score pos_h=9.9'  'pos_h<=10.0'
    t 1 'and in the other direction'  'score pos_h=10.0' 'pos_h<=9.9'
    t 0 'negatives'                   'score yaw0=-94.61' 'yaw0>=-100'

    # `pos_h` and `pos_h_max` are both keys on the score line. A prefix match would read the
    # excursion where the RMSE was asked for, and pass a ceiling the filter breached.
    t 1 'a key is not a prefix' 'score pos_h_max=125.000' 'pos_h<=200'

    t 1 'missing key'    "$score" 'nees_vel<=1'
    t 1 'not a number'   "$score" 'nees_att<=0.05'
    t 1 'malformed pair' "$score" 'pos_h'

    # A value holding a glob character, checked from a directory where it matches files.
    # Both loops split on an unquoted expansion, so without `set -f` both sides expand --
    # and two matches is what makes that visible rather than self-cancelling: the line keeps
    # the first match while the expectations become two pairs, the second of which nothing
    # on the line satisfies. No `score` key can hold a glob character today, which is why
    # the guard needs a fixture rather than a reader's memory.
    local sandbox status
    sandbox=$(mktemp -d)
    touch "$sandbox/pos_h=12" "$sandbox/pos_h=13"
    status=0
    (
        cd "$sandbox" || exit 1
        compare_pairs 'score pos_h=1* scored=5' 'pos_h=1* scored>=5' 2>/dev/null
    ) || status=$?
    if [ "$status" = 0 ]; then
        passed=$((passed + 1))
    else
        failed=$((failed + 1))
        echo "  FAIL  a globbing value was expanded before it was compared" >&2
    fi
    rm -rf "$sandbox"

    # #17: lowering one ceiling fails naming the pair.
    local message
    message=$(compare_pairs "$score" 'pos_h<=100.000' mission 2>&1 >/dev/null) || true
    case "$message" in
        *mission*pos_h=101.112*pos_h'<='100.000*) passed=$((passed + 1)) ;;
        *) failed=$((failed + 1)); echo "  FAIL  breach message: $message" >&2 ;;
    esac

    echo "expect.sh: $passed passed, $failed failed"
    [ "$failed" = 0 ]
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    set -euo pipefail
    case "${1:-}" in
        --self-test) self_test ;;
        -h|--help) sed -n '2,/^# Read/p' "$0" | sed -e 's/^#//' -e 's/^ //' -e '$d' ;;
        *) echo "expect.sh is sourced for compare_pairs; --self-test runs its fixtures" >&2; exit 2 ;;
    esac
fi
