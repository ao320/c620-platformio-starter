#import <Foundation/Foundation.h>
#import <IOKit/hid/IOHIDManager.h>
static unsigned changes = 0;
static void valueCallback(void *context, IOReturn result, void *sender, IOHIDValueRef value) {
  IOHIDElementRef element = IOHIDValueGetElement(value);
  uint32_t page = IOHIDElementGetUsagePage(element), usage = IOHIDElementGetUsage(element);
  if (++changes <= 100) printf("value page=%u usage=%u value=%ld\n", page, usage, (long)IOHIDValueGetIntegerValue(value));
  fflush(stdout);
}
int main(int argc, char **argv) {
  @autoreleasepool {
    int seconds = argc > 1 ? atoi(argv[1]) : 3;
    IOHIDManagerRef manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    // Only Nintendo-compatible Bluetooth gamepad; never keyboard/input monitoring.
    NSDictionary *match = @{@kIOHIDVendorIDKey:@1406, @kIOHIDProductIDKey:@8201,
                            @kIOHIDDeviceUsagePageKey:@1, @kIOHIDDeviceUsageKey:@5};
    IOHIDManagerSetDeviceMatching(manager, (__bridge CFDictionaryRef)match);
    IOHIDManagerRegisterInputValueCallback(manager,valueCallback,NULL);
    IOHIDManagerScheduleWithRunLoop(manager,CFRunLoopGetCurrent(),kCFRunLoopDefaultMode);
    IOReturn result=IOHIDManagerOpen(manager,kIOHIDOptionsTypeNone);
    CFSetRef devices=IOHIDManagerCopyDevices(manager);
    printf("open=0x%08x devices=%ld\n",result,devices?(long)CFSetGetCount(devices):0);fflush(stdout);
    if(devices)CFRelease(devices);
    if(result==kIOReturnSuccess)CFRunLoopRunInMode(kCFRunLoopDefaultMode,seconds,false);
    printf("inputValues=%u\n",changes);fflush(stdout);
    IOHIDManagerClose(manager,kIOHIDOptionsTypeNone);CFRelease(manager);
  }
}
