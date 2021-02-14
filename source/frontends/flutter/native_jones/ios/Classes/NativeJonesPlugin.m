#import "NativeJonesPlugin.h"
#if __has_include(<native_jones/native_jones-Swift.h>)
#import <native_jones/native_jones-Swift.h>
#else
// Support project import fallback if the generated compatibility header
// is not copied when this plugin is created as a library.
// https://forums.swift.org/t/swift-static-libraries-dont-copy-generated-objective-c-header/19816
#import "native_jones-Swift.h"
#endif

@implementation NativeJonesPlugin
+ (void)registerWithRegistrar:(NSObject<FlutterPluginRegistrar>*)registrar {
  [SwiftNativeJonesPlugin registerWithRegistrar:registrar];
}
@end
