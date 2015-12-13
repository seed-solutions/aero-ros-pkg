echo -e "
function aerocd() {
    if [[ \$1 == \"\" ]]
    then
        roscd aero_startup
        return
    fi
    num=\$(find \$(rospack find aero_startup)/ -name \"\$1*\" | wc -l)
    if [[ \$num -eq 0 ]]
    then
        echo \"not found\"
        return
    fi
    file=\$(find \$(rospack find aero_startup)/ -name \"\$1*\" | sed 's:/[^/]*\$::' | sort -u)
    num=\$(find \$(rospack find aero_startup)/ -name \"\$1*\" | sed 's:/[^/]*\$::' | sort -u | wc -l)
    if [[ \"\$num\" -gt \"1\" ]]
    then
        if [[ \$2 == \"\" ]]
        then
            echo -e \"which?\\\n\${file}\"
            return
        fi
        file=\$(find \$(rospack find aero_startup)/ -name \"\$1*\" | sed 's:/[^/]*\$::' | sort -u | sed -n \"\${2}p\")
    fi
    cd \$file
}" >> $HOME/.bashrc
