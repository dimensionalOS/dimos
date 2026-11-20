# cpu_snap.sh <label>: per-process and per-thread CPU on GOS, AOS and NOS, 3 x 5 s samples each (top -H), top 15 lines
label=$1
snap() { top -b -n 4 -d 5 $1 -w 200 | awk '/^top/{n++} n>=2' | grep -vE "^(Tasks|MiB|Threads)" | awk '/^top/{c=0} {c++} c<=22' ; }
echo "=== $label GOS processes"; snap "" | grep -E "^top|Cpu|^ *[0-9]" | head -60
echo "=== $label GOS threads"; snap -H | grep -E "^top|^ *[0-9]" | head -60
for b in 10.21.31.103 10.21.31.106; do
    echo "=== $label $b processes"; ssh $b "$(declare -f snap); snap ''" | grep -E "^top|Cpu|^ *[0-9]" | head -60
    echo "=== $label $b threads"; ssh $b "$(declare -f snap); snap -H" | grep -E "^top|^ *[0-9]" | head -60
done
