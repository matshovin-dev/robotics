#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ApplicationServices/ApplicationServices.h>
#include <Carbon/Carbon.h>

// Kompilér: clang -o focus focus_wind.c -framework ApplicationServices
// -framework Carbon
// osascript -e 'id of app "Firefox"' finner bundle ID for apper

// Numpad-taster enum
enum {
	NP_0 = 0,
	NP_1,
	NP_2,
	NP_3,
	NP_4,
	NP_5,
	NP_6,
	NP_7,
	NP_8,
	NP_9,
	NP_PLUS,
	NP_MINUS,
	NP_MULT,
	NP_DIV,
	NP_EQUALS,
	NP_DOT,
	NP_ENTER,
	NP_CLEAR,
	NP_COUNT
};

// Bundle IDs for apper (index mapper til numpad-taster via enum)
// Bruk semikolon for å fokusere flere vinduer samtidig: "win:A;win:B;win:C"
static const char *apps[NP_COUNT] = { [NP_0] = "com.microsoft.VSCode",
				      [NP_1] = NULL,
				      [NP_2] = NULL,
				      [NP_3] = NULL,
				      [NP_4] = NULL,
				      [NP_5] = "win:C:;win:B:;win:Ste",
				      [NP_6] = "win:C:",
				      [NP_7] = NULL,
				      [NP_8] = NULL,
				      [NP_9] = NULL,
				      [NP_PLUS] = NULL,
				      [NP_MINUS] = "proc:Finder",
				      [NP_MULT] = "org.mozilla.firefox",
				      [NP_DIV] = NULL,
				      [NP_EQUALS] = "cmd:vscode_zen",
				      [NP_DOT] = "com.apple.Terminal",
				      [NP_ENTER] = NULL,
				      [NP_CLEAR] = NULL };

// Keycodes for tallene 1-4 på Mac tastatur (krever Ctrl)
static const CGKeyCode keycodes[] = {
	[1] = kVK_ANSI_1,  // 18
	[2] = kVK_ANSI_2,  // 19
	[3] = kVK_ANSI_3,  // 20
	[4] = kVK_ANSI_4  // 21
};

// Numpad keycodes (fungerer uten modifier)
static const CGKeyCode numpad_keycodes[NP_COUNT] = {
	[NP_0] = kVK_ANSI_Keypad0,  // 82
	[NP_1] = kVK_ANSI_Keypad1,  // 83
	[NP_2] = kVK_ANSI_Keypad2,  // 84
	[NP_3] = kVK_ANSI_Keypad3,  // 85
	[NP_4] = kVK_ANSI_Keypad4,  // 86
	[NP_5] = kVK_ANSI_Keypad5,  // 87
	[NP_6] = kVK_ANSI_Keypad6,  // 88
	[NP_7] = kVK_ANSI_Keypad7,  // 89
	[NP_8] = kVK_ANSI_Keypad8,  // 91
	[NP_9] = kVK_ANSI_Keypad9,  // 92
	[NP_PLUS] = kVK_ANSI_KeypadPlus,  // 69
	[NP_MINUS] = kVK_ANSI_KeypadMinus,  // 78
	[NP_MULT] = kVK_ANSI_KeypadMultiply,  // 67
	[NP_DIV] = kVK_ANSI_KeypadDivide,  // 75
	[NP_EQUALS] = kVK_ANSI_KeypadEquals,  // 81
	[NP_DOT] = kVK_ANSI_KeypadDecimal,  // 65
	[NP_ENTER] = kVK_ANSI_KeypadEnter,  // 76
	[NP_CLEAR] = kVK_ANSI_KeypadClear  // 71
};

// Prefiks for ulike typer targets
#define PREFIX_PROC "proc:"  // Prosessnavn (f.eks. "proc:mitt_program")
#define PREFIX_WIN "win:"  // Vindustittel (f.eks. "win:Plot Window")
#define PREFIX_CMD "cmd:"  // Kommando (f.eks. "cmd:vscode_zen")

// Global event tap referanse for re-enabling
static CFMachPortRef g_event_tap = NULL;

// Send tastetrykk
static void send_key(CGKeyCode key, CGEventFlags flags)
{
	CGEventRef down = CGEventCreateKeyboardEvent(NULL, key, true);
	CGEventRef up = CGEventCreateKeyboardEvent(NULL, key, false);
	if (flags) {
		CGEventSetFlags(down, flags);
		CGEventSetFlags(up, flags);
	}
	CGEventPost(kCGHIDEventTap, down);
	CGEventPost(kCGHIDEventTap, up);
	CFRelease(down);
	CFRelease(up);
}

// Utfør kommando
static void run_command(const char *cmd_name)
{
	if (strcmp(cmd_name, "vscode_zen") == 0) {
		// VS Code Zen Mode: Cmd+K, deretter Z
		// Først fokuser VS Code
		pid_t pid = 0;
		// Finn VS Code PID
		CFArrayRef apps = CGWindowListCopyWindowInfo(
			kCGWindowListOptionOnScreenOnly |
				kCGWindowListExcludeDesktopElements,
			kCGNullWindowID);
		if (apps) {
			CFIndex count = CFArrayGetCount(apps);
			for (CFIndex i = 0; i < count && pid == 0; i++) {
				CFDictionaryRef win =
					CFArrayGetValueAtIndex(apps, i);
				CFStringRef name = CFDictionaryGetValue(
					win, kCGWindowOwnerName);
				CFNumberRef pid_ref = CFDictionaryGetValue(
					win, kCGWindowOwnerPID);
				if (name && pid_ref) {
					char buf[256];
					if (CFStringGetCString(
						    name, buf, sizeof(buf),
						    kCFStringEncodingUTF8)) {
						if (strstr(buf, "Code") !=
						    NULL) {
							CFNumberGetValue(
								pid_ref,
								kCFNumberIntType,
								&pid);
						}
					}
				}
			}
			CFRelease(apps);
		}

		if (pid > 0) {
			AXUIElementRef app = AXUIElementCreateApplication(pid);
			if (app) {
				AXUIElementSetAttributeValue(
					app, kAXFrontmostAttribute,
					kCFBooleanTrue);
				CFRelease(app);
			}
			usleep(50000);	// Vent 50ms for fokus
		}

		// Send Cmd+K
		send_key(kVK_ANSI_K, kCGEventFlagMaskCommand);
		usleep(100000);	 // Vent 100ms
		// Send Z
		send_key(kVK_ANSI_Z, 0);
		printf("VS Code Zen Mode toggled\n");
	} else {
		printf("Ukjent kommando: %s\n", cmd_name);
	}
}

// Aktiver app via PID med Accessibility API
static void activate_by_pid(pid_t pid)
{
	AXUIElementRef app = AXUIElementCreateApplication(pid);
	if (app) {
		AXUIElementSetAttributeValue(app, kAXFrontmostAttribute,
					     kCFBooleanTrue);
		CFRelease(app);
	}
}

// Finn PID fra bundle ID
static pid_t find_pid_by_bundle(const char *bundle_id)
{
	CFArrayRef apps = CGWindowListCopyWindowInfo(
		kCGWindowListOptionOnScreenOnly |
			kCGWindowListExcludeDesktopElements,
		kCGNullWindowID);
	if (!apps)
		return 0;

	pid_t result = 0;
	CFIndex count = CFArrayGetCount(apps);

	for (CFIndex i = 0; i < count && result == 0; i++) {
		CFDictionaryRef win = CFArrayGetValueAtIndex(apps, i);
		CFNumberRef pid_ref =
			CFDictionaryGetValue(win, kCGWindowOwnerPID);
		if (!pid_ref)
			continue;

		pid_t pid;
		CFNumberGetValue(pid_ref, kCFNumberIntType, &pid);

		// Hent bundle ID for denne PID-en
		ProcessSerialNumber psn;
		if (GetProcessForPID(pid, &psn) == noErr) {
			CFDictionaryRef info = ProcessInformationCopyDictionary(
				&psn,
				kProcessDictionaryIncludeAllInformationMask);
			if (info) {
				CFStringRef bid = CFDictionaryGetValue(
					info, kCFBundleIdentifierKey);
				if (bid) {
					char buf[256];
					if (CFStringGetCString(
						    bid, buf, sizeof(buf),
						    kCFStringEncodingUTF8)) {
						if (strcmp(buf, bundle_id) ==
						    0) {
							result = pid;
						}
					}
				}
				CFRelease(info);
			}
		}
	}
	CFRelease(apps);
	return result;
}

// Finn PID fra prosessnavn
static pid_t find_pid_by_name(const char *proc_name)
{
	CFArrayRef apps = CGWindowListCopyWindowInfo(
		kCGWindowListOptionOnScreenOnly |
			kCGWindowListExcludeDesktopElements,
		kCGNullWindowID);
	if (!apps)
		return 0;

	pid_t result = 0;
	CFIndex count = CFArrayGetCount(apps);

	for (CFIndex i = 0; i < count && result == 0; i++) {
		CFDictionaryRef win = CFArrayGetValueAtIndex(apps, i);
		CFStringRef name =
			CFDictionaryGetValue(win, kCGWindowOwnerName);
		CFNumberRef pid_ref =
			CFDictionaryGetValue(win, kCGWindowOwnerPID);
		if (!name || !pid_ref)
			continue;

		char buf[256];
		if (CFStringGetCString(name, buf, sizeof(buf),
				       kCFStringEncodingUTF8)) {
			if (strstr(buf, proc_name) != NULL) {
				CFNumberGetValue(pid_ref, kCFNumberIntType,
						 &result);
			}
		}
	}
	CFRelease(apps);
	return result;
}

// Finn PID fra vindustittel
static pid_t find_pid_by_window_title(const char *title)
{
	CFArrayRef apps = CGWindowListCopyWindowInfo(
		kCGWindowListOptionOnScreenOnly |
			kCGWindowListExcludeDesktopElements,
		kCGNullWindowID);
	if (!apps)
		return 0;

	pid_t result = 0;
	CFIndex count = CFArrayGetCount(apps);

	for (CFIndex i = 0; i < count && result == 0; i++) {
		CFDictionaryRef win = CFArrayGetValueAtIndex(apps, i);
		CFStringRef name = CFDictionaryGetValue(win, kCGWindowName);
		CFNumberRef pid_ref =
			CFDictionaryGetValue(win, kCGWindowOwnerPID);
		if (!name || !pid_ref)
			continue;

		char buf[512];
		if (CFStringGetCString(name, buf, sizeof(buf),
				       kCFStringEncodingUTF8)) {
			if (strstr(buf, title) != NULL) {
				CFNumberGetValue(pid_ref, kCFNumberIntType,
						 &result);
			}
		}
	}
	CFRelease(apps);
	return result;
}

// Debug: list alle vinduer
static void list_all_windows(void)
{
	CFArrayRef apps = CGWindowListCopyWindowInfo(
		kCGWindowListOptionOnScreenOnly |
			kCGWindowListExcludeDesktopElements,
		kCGNullWindowID);
	if (!apps)
		return;

	printf("\n=== Alle åpne vinduer ===\n");
	CFIndex count = CFArrayGetCount(apps);
	for (CFIndex i = 0; i < count; i++) {
		CFDictionaryRef win = CFArrayGetValueAtIndex(apps, i);
		CFStringRef owner =
			CFDictionaryGetValue(win, kCGWindowOwnerName);
		CFStringRef title = CFDictionaryGetValue(win, kCGWindowName);
		CFNumberRef pid_ref =
			CFDictionaryGetValue(win, kCGWindowOwnerPID);

		char owner_buf[256] = "(null)";
		char title_buf[256] = "(null)";
		pid_t pid = 0;

		if (owner)
			CFStringGetCString(owner, owner_buf, sizeof(owner_buf),
					   kCFStringEncodingUTF8);
		if (title)
			CFStringGetCString(title, title_buf, sizeof(title_buf),
					   kCFStringEncodingUTF8);
		if (pid_ref)
			CFNumberGetValue(pid_ref, kCFNumberIntType, &pid);

		if (strlen(title_buf) > 0 && strcmp(title_buf, "(null)") != 0) {
			printf("  [%d] %s: \"%s\"\n", pid, owner_buf,
			       title_buf);
		}
	}
	printf("=========================\n\n");
	CFRelease(apps);
}

static void activate_single_target(const char *target)
{
	pid_t pid = 0;

	if (strncmp(target, PREFIX_CMD, strlen(PREFIX_CMD)) == 0) {
		const char *cmd_name = target + strlen(PREFIX_CMD);
		run_command(cmd_name);
		return;
	} else if (strncmp(target, PREFIX_PROC, strlen(PREFIX_PROC)) == 0) {
		const char *proc_name = target + strlen(PREFIX_PROC);
		pid = find_pid_by_name(proc_name);
		printf("Aktiverer prosess: %s (PID: %d)\n", proc_name, pid);
	} else if (strncmp(target, PREFIX_WIN, strlen(PREFIX_WIN)) == 0) {
		const char *win_title = target + strlen(PREFIX_WIN);
		pid = find_pid_by_window_title(win_title);
		printf("Aktiverer vindu: %s (PID: %d)\n", win_title, pid);
	} else {
		pid = find_pid_by_bundle(target);
		printf("Aktiverer: %s (PID: %d)\n", target, pid);
	}

	if (pid > 0) {
		activate_by_pid(pid);
	} else {
		printf("  Fant ikke target!\n");
	}
}

// Aktiverer ett eller flere targets (semikolon-separert)
static void activate_app(const char *targets)
{
	char buf[512];
	strncpy(buf, targets, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	char *token = strtok(buf, ";");
	while (token != NULL) {
		activate_single_target(token);
		token = strtok(NULL, ";");
	}
}

static CGEventRef event_callback(CGEventTapProxy proxy, CGEventType type,
				 CGEventRef event, void *user_info)
{
	if (type == kCGEventKeyDown) {
		CGEventFlags flags = CGEventGetFlags(event);
		CGKeyCode keycode = (CGKeyCode)CGEventGetIntegerValueField(
			event, kCGKeyboardEventKeycode);

		// Sjekk numpad-taster (ingen modifier nødvendig)
		for (int i = 0; i < NP_COUNT; i++) {
			if (keycode == numpad_keycodes[i] && apps[i] != NULL) {
				activate_app(apps[i]);
				return NULL;  // Blokkér numpad-tasten
			}
		}

		// Sjekk om Ctrl er holdt nede for vanlig tastatur
		bool ctrl_held = (flags & kCGEventFlagMaskControl) != 0;
		if (ctrl_held) {
			for (int i = 1; i <= 4; i++) {
				if (keycode == keycodes[i] && apps[i] != NULL) {
					activate_app(apps[i]);
					return NULL;  // Blokkér Ctrl+1-4
				}
			}
		}
	}

	// Re-enable hvis tap ble disabled
	if (type == kCGEventTapDisabledByTimeout ||
	    type == kCGEventTapDisabledByUserInput) {
		if (g_event_tap) {
			CGEventTapEnable(g_event_tap, true);
		}
	}

	return event;
}

int main(void)
{
	printf("Focus Window - Tastatur-mapper\n");
	printf("Numpad: 0=VSCode, 5=C:/B:/Ste vinduer, 6=C: vindu\n");
	printf("        -=Finder, *=Firefox, .=Terminal, ==Zen\n");
	printf("Tastatur: Ctrl+1-4 (samme som over)\n");
	printf("\nTarget-format i apps[]:\n");
	printf("  \"com.app.bundle\"     - Bundle ID\n");
	printf("  \"proc:programnavn\"   - Prosessnavn\n");
	printf("  \"win:vindustittel\"   - Vindustittel\n");
	printf("  \"a;b;c\"              - Flere targets samtidig\n");
	printf("\nCtrl+C for å avslutte\n");

	list_all_windows();

	CGEventMask mask = CGEventMaskBit(kCGEventKeyDown);
	g_event_tap = CGEventTapCreate(
		kCGSessionEventTap, kCGHeadInsertEventTap,
		kCGEventTapOptionDefault, mask, event_callback, NULL);

	if (!g_event_tap) {
		fprintf(stderr, "Feil: Kunne ikke opprette event tap.\n");
		fprintf(stderr, "Gi Accessibility-tilgang i:\n");
		fprintf(stderr,
			"  System Settings -> Privacy & Security -> Accessibility\n");
		return 1;
	}

	CFRunLoopSourceRef source = CFMachPortCreateRunLoopSource(
		kCFAllocatorDefault, g_event_tap, 0);
	CFRunLoopAddSource(CFRunLoopGetCurrent(), source,
			   kCFRunLoopCommonModes);
	CGEventTapEnable(g_event_tap, true);

	printf("Lytter på tastetrykk...\n");
	CFRunLoopRun();

	CFRelease(source);
	CFRelease(g_event_tap);
	return 0;
}
