FILE(REMOVE_RECURSE
  "CMakeFiles/localization_generate_messages_cpp"
  "devel/include/localization/encoder.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/localization_generate_messages_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
