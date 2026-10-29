#import <Foundation/Foundation.h>
#import <GameController/GameController.h>
// Enumeration only: no discovery, capture, vibration, or device configuration.
int main(void) {
  @autoreleasepool {
    [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:2]];
    NSMutableArray *items = [NSMutableArray array];
    for (GCController *c in [GCController controllers]) {
      [items addObject:@{@"vendorName": c.vendorName ?: @"unknown",
                         @"productCategory": c.productCategory ?: @"unknown",
                         @"extendedGamepad": @(c.extendedGamepad != nil),
                         @"microGamepad": @(c.microGamepad != nil)}];
    }
    NSData *data = [NSJSONSerialization dataWithJSONObject:items options:NSJSONWritingPrettyPrinted error:nil];
    puts([[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding].UTF8String);
  }
}
