#import <Foundation/Foundation.h>
#import <IOKit/hid/IOHIDManager.h>
static void reportCallback(void *context, IOReturn result, void *sender, IOHIDReportType type,
                           uint32_t reportID, uint8_t *report, CFIndex length) {
  if (result != kIOReturnSuccess || type != kIOHIDReportTypeInput || length <= 0 || length > 512) return;
  printf("{\"reportId\":%u,\"hex\":\"",reportID);
  for (CFIndex i=0;i<length;++i)printf("%02x",report[i]);
  puts("\"}");fflush(stdout);
}
int main(int argc, char **argv) {
 @autoreleasepool {
  IOHIDManagerRef manager=IOHIDManagerCreate(kCFAllocatorDefault,kIOHIDOptionsTypeNone);
  NSDictionary *match=@{@kIOHIDVendorIDKey:@1406,@kIOHIDProductIDKey:@8201,
                        @kIOHIDDeviceUsagePageKey:@1,@kIOHIDDeviceUsageKey:@5,
                        @kIOHIDTransportKey:@"Bluetooth"};
  IOHIDManagerSetDeviceMatching(manager,(__bridge CFDictionaryRef)match);
  IOHIDManagerRegisterInputReportCallback(manager,reportCallback,NULL);
  IOHIDManagerScheduleWithRunLoop(manager,CFRunLoopGetCurrent(),kCFRunLoopDefaultMode);
  IOReturn result=IOHIDManagerOpen(manager,kIOHIDOptionsTypeNone);
  printf("{\"openResult\":%u}\n",result);fflush(stdout);
  if(result==kIOReturnSuccess) {
    if(argc>1)CFRunLoopRunInMode(kCFRunLoopDefaultMode,atoi(argv[1]),false);
    else CFRunLoopRun();
  }
  IOHIDManagerClose(manager,kIOHIDOptionsTypeNone);CFRelease(manager);
 }
}
