cpplint $( find ../. -name \*.hpp -or -name \*.cpp -or -name \*.h | grep -vE -e "^.././BashScripts" -e "^.././build" -e "^.././cmake" -e "^.././docs" -e "^.././doxygen" -e "^.././vendor" )
