#pragma once
void umb_test_lwip_begin(void);
void umb_test_lwip_end(void);
#define cyw43_arch_lwip_begin umb_test_lwip_begin
#define cyw43_arch_lwip_end umb_test_lwip_end
