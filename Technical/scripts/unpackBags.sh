#/bin/bash
#  Correct glob use:
#  Always use for-loop, prefix glob, check if exists file.
for file in *.bag; do         # Use ./* ... NEVER bare *
  if [ -e "$file" ] ; then   # Check whether file exists.
      echo "Processing bag: $file"
      theDir=(${file//./ })
      #echo "Making directory:" ${theDir[0]}
      mkdir -p ${theDir[0]}
      rosrun data_extraction extract_all.py -b ./$file -o ${theDir[0]}
  fi
done
