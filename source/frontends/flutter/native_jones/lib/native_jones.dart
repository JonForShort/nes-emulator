
import 'dart:async';

import 'package:flutter/services.dart';

class NativeJones {
  static const MethodChannel _channel =
      const MethodChannel('native_jones');

  static Future<String> get platformVersion async {
    final String version = await _channel.invokeMethod('getPlatformVersion');
    return version;
  }
}
