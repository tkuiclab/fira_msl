FILE(REMOVE_RECURSE
  "CMakeFiles/localization_generate_messages_py"
  "devel/lib/python2.7/dist-packages/localization/srv/_encoder.py"
  "devel/lib/python2.7/dist-packages/localization/srv/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/localization_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
