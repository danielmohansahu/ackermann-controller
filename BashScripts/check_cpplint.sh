echo "Returning CPPLint check, level 4 and 5 errors only."
cpplint --verbose=4 $( find ../. -name \*.hpp -or -name \*.cpp -or -name \*.h | grep -vE -e "^.././BashScripts" -e "^.././build" -e "^.././cmake" -e "^.././docs" -e "^.././doxygen" -e "^.././vendor" )
