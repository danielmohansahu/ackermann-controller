cpplint $( find . -name \*.hpp -or -name \*.cpp -or -name \*.h | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./doxygen/" )
