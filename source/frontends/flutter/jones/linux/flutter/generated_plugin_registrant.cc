//
//  Generated file. Do not edit.
//

#include "generated_plugin_registrant.h"

#include <native_jones/native_jones_plugin.h>

void fl_register_plugins(FlPluginRegistry* registry) {
  g_autoptr(FlPluginRegistrar) native_jones_registrar =
      fl_plugin_registry_get_registrar_for_plugin(registry, "NativeJonesPlugin");
  native_jones_plugin_register_with_registrar(native_jones_registrar);
}
