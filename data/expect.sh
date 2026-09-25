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
# Four forms, because a corpus expectation and a benchmark ceiling want different things:
#
#   key=value    exact, string. What the corpus pins: `status=Healthy`, `rate=250`.
#   key<=value   numeric ceiling. An error the filter must not exceed.
#   key>=value   numeric floor. A goodness key -- `in3s`, `scored`.
#   key=lo..hi   numeric range, both bounds inclusive. What a statistic wants.
#
# The range exists because pinning a statistic to its last digit says "this number" where the
# claim is "this receiver reports six times the accuracy its own solutions support". A range
# states the claim, and survives a filter change that moves the digit without moving the
# finding. `nis_gnss_vel=6.0..10.0` is an assertion about a receiver; `nis_gnss_vel=7.9089` is
# an assertion about a build.
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
    local line=$1 expect=$2 label=${3:-} rc=0 pair key op want want_lo want_hi got status
    # Both loops here split on an unquoted expansion -- this one over the expectations, the
    # one in `pair_value` over the line -- so both are open to pathname expansion. Disabled
    # for the whole function, which covers `pair_value` too, since the option is global.
    local restore_glob=0
    case $- in *f*) ;; *) restore_glob=1; set -f ;; esac
    for pair in $expect; do
        case "$pair" in
            # Arm order is load-bearing: all four forms contain an `=`, so each narrower
            # pattern has to precede the bare `=` or that one swallows it. A range landing in
            # the `=` arm compares `1.0..2.0` as a string and fails every value it should
            # pass. `..` goes after `<=` and `>=` so that a `pos_h<=1..2` nobody meant to
            # write reports its value as not a number rather than hunting for a `pos_h<` key.
            *'<='*) key=${pair%%<=*} op='<=' want=${pair##*<=} ;;
            *'>='*) key=${pair%%>=*} op='>=' want=${pair##*>=} ;;
            *=*..*) key=${pair%%=*}  op='..' want=${pair#*=}
                    want_lo=${want%%..*} want_hi=${want##*..} ;;
            *=*)    key=${pair%%=*}  op='='  want=${pair#*=} ;;
            *) echo "  MALFORMED  ${label:+$label: }$pair has no =, <=, >= or .." >&2; rc=1; continue ;;
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
        # A range holds when both of its bounds hold, so it is two `numeric_ok` calls and not
        # a second comparator: the observed value and both endpoints go through the one
        # numeric check, which is what keeps `nees_pos=none` from reading as zero here too.
        # `&&` yields the left status when the left fails and the right one otherwise, so a
        # garbage endpoint still surfaces as 2 rather than as a breach.
        #
        # The alternative -- rewriting `key=lo..hi` into `key>=lo` and `key<=hi` and deleting
        # this branch -- was not taken: the diagnostic would then name a pair nobody wrote,
        # and the contract of every message below is that it can be copied back into the file
        # it came from.
        if [ "$op" = '..' ]; then
            numeric_ok "$got" '>=' "$want_lo" && numeric_ok "$got" '<=' "$want_hi"
        else
            numeric_ok "$got" "$op" "$want"
        fi
        status=$?
        case $status in
            0) ;;
            1) echo "  BREACH     ${label:+$label: }$key=$got, wanted $pair" >&2; rc=1 ;;
            *) echo "  NOT A NUMBER ${label:+$label: }$key=$got cannot be compared against $pair" >&2; rc=1 ;;
        esac
    done
    [ "$restore_glob" = 1 ] && set +f
    return $rc
}

# pin_pairs <line>
#
# The inverse of compare_pairs: expectations for a `summary` line, by the rule the header of
# data/manifest.txt states, so a new entry's forty-odd pairs are derived rather than typed.
# Exact for every key, except the measured statistics -- the four consistency families and
# what the vehicle did -- which get a range: the value plus or minus the larger of 1 % and one
# unit in its last printed place, rounded outward to that place. A zero or a word stays
# exact, and a mean innovation inside a thousandth is pinned to +/-0.001, since what it
# claims is the absence of an offset. `epochs=` is left out: it counts rows, so it would
# pin the converter's output rather than the filter's.
pin_pairs() {
    awk -v line="$1" '
        function floor(x) { return (x == int(x) || x > 0) ? int(x) : int(x) - 1 }
        function ceil(x)  { return -floor(-x) }
        # `text` is the value as printed, which says how many places to round to; `v` is the
        # same value as a number. substr() yields a string, and mawk and gawk compare a
        # string against a constant as strings, where "-0.011616" > "-0.001".
        function band(key, text,    v, places, unit, delta, scale, format) {
            v = text + 0
            places = index(text, ".") ? length(text) - index(text, ".") : 0
            format = "%." places "f"
            if (key ~ /^nu_/ && v < 0.001 && v > -0.001)
                return sprintf(format ".." format, -0.001, 0.001)
            scale = 10 ^ places
            unit = 1 / scale
            delta = (v < 0 ? -v : v) * 0.01
            if (delta < unit) delta = unit
            # 1e-6 of a unit absorbs the binary error in `value * scale`; without it a bound
            # that lands on the grid, as 0.0006 - 0.0001 does, rounds out one more unit.
            return sprintf(format ".." format,
                           floor((v - delta) * scale + 1e-6) / scale,
                           ceil((v + delta) * scale - 1e-6) / scale)
        }
        BEGIN {
            number = "^-?[0-9]+([.][0-9]+)?$"
            statistic = "^(nis_|nu_|acf1_)|^(extent|speed_max|tilt_max)$"
            n = split(line, words, " ")
            out = ""
            for (i = 1; i <= n; i++) {
                eq = index(words[i], "=")
                if (!eq) continue
                key = substr(words[i], 1, eq - 1)
                value = substr(words[i], eq + 1)
                if (key == "epochs") continue
                pair = words[i]
                if (key ~ statistic && value ~ number && value + 0 != 0)
                    pair = key "=" band(key, value)
                out = out (out == "" ? "" : " ") pair
            }
            print out
        }'
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

    # #4: a two-sided bound. The first fixture is the one that catches the `..` arm placed
    # after the bare `=`, where the pair string-compares `1.0..2.0` against `1.5` and fails.
    t 0 'inside the range'        'score nis=1.5'  'nis=1.0..2.0'
    t 1 'below the range'         'score nis=0.9'  'nis=1.0..2.0'
    t 1 'above the range'         'score nis=2.1'  'nis=1.0..2.0'
    t 0 'range bounds are inclusive at the bottom' 'score nis=1.0' 'nis=1.0..2.0'
    t 0 'and at the top'                           'score nis=2.0' 'nis=1.0..2.0'
    # Lexically 9.9 sits above 10.0, so a range compared as strings passes this and then
    # passes everything else it should refuse.
    t 0 'a range compares as numbers' 'score nis=9.9' 'nis=1.0..10.0'
    t 0 'a range over negatives'      'score nu=-0.02' 'nu=-0.10..0.10'
    # Both endpoints go through `numeric_ok`, so the `none` the harness prints for a block
    # that did not invert cannot read as zero and clear a bound the filter never met.
    t 1 'a range against none'        "$score" 'nees_att=0.0..1.0'
    # An unterminated range is the shape a half-finished edit leaves. Read as a one-sided
    # bound it would silently stop checking the end somebody deleted.
    t 1 'no upper bound'              'score nis=1.5' 'nis=1.0..'
    t 1 'no lower bound'              'score nis=1.5' 'nis=..2.0'

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

    # pin_pairs, against bands data/manifest.txt carries for values its prose quotes.
    # p <name> <line> <wanted expectations>
    p() {
        local got
        got=$(pin_pairs "$2")
        if [ "$got" = "$3" ]; then
            passed=$((passed + 1))
        else
            failed=$((failed + 1))
            echo "  FAIL  pin $1: got '$got', wanted '$3'" >&2
        fi
    }
    p '1 % of the value'        'summary nis_gnss_vel=7.5592' 'nis_gnss_vel=7.4836..7.6348'
    p 'negative, outward'       'summary nu_baro_d=-0.011616' 'nu_baro_d=-0.011733..-0.011499'
    # 1 % of 0.0035 is under a unit of 0.0001, so the unit is the half-width.
    p 'one unit'                'summary nis_over95_mag=0.0035' 'nis_over95_mag=0.0034..0.0036'
    # Bounds that land on the grid, where `value * scale` carries binary error: without the
    # 1e-6 these read 0.0004..0.0007 and 0.0004..0.0007 -- a second unit out on one side.
    p 'on the grid, below'      'summary nis_over95_mag=0.0006' 'nis_over95_mag=0.0005..0.0007'
    p 'on the grid, above'      'summary nis_over95_mag=0.0005' 'nis_over95_mag=0.0004..0.0006'
    p 'no offset'               'summary nu_mag_yaw=0.000400' 'nu_mag_yaw=-0.001000..0.001000'
    p 'no offset, negative'     'summary nu_mag_yaw=-0.000400' 'nu_mag_yaw=-0.001000..0.001000'
    p 'the thousandth is nu only' 'summary acf1_mag=0.0004' 'acf1_mag=0.0003..0.0005'
    p 'zero and none stay exact' 'summary nis_over95_baro=0.0000 nis_baro=none' \
        'nis_over95_baro=0.0000 nis_baro=none'
    # Exact keys stay exact however decimal they look, and a key is matched whole: a
    # prefix match on `nu` would band `nudge` and one on `extent` would band `extent_max`.
    p 'exact keys, whole names' 'summary yaw0=-107.33 attitude_lost=3.84 extent_max=5.0 status=Healthy' \
        'yaw0=-107.33 attitude_lost=3.84 extent_max=5.0 status=Healthy'
    p 'what the vehicle did'    'summary extent=112.6 tilt_max=41.1' 'extent=111.4..113.8 tilt_max=40.6..41.6'
    p 'epochs is not pinned'    'summary rate=250 epochs=16079 status=Healthy' 'rate=250 status=Healthy'

    # What --pin prints has to pass --check against the line it came from.
    local line='summary rate=250 nis_mag=0.0907 nu_baro_d=-0.011616 nu_mag_yaw=-0.000400 extent=112.6 status=Healthy'
    t 0 'pin_pairs round-trips' "$line" "$(pin_pairs "$line")"

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
