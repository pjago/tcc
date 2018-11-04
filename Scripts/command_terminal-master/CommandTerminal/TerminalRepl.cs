using UnityEngine;
using System.Collections;
using UnityEngine.Assertions;
using clojure.lang;
using Arcadia;
using Util = Arcadia.Util;
using System;
using System.Net;
using System.Text;
using System.Net.Sockets;
using System.Collections.Generic;
using System.Threading;
using System.Runtime.InteropServices;
using System.IO;
using System.Linq;

namespace CommandTerminal
{
    public enum TerminalReplState
    {
        Close,
        OpenSmall,
        OpenFull
    }

    public class TerminalRepl : MonoBehaviour
    {
        [Header("Window")]
        [Range(0, 1)]
        [SerializeField]
        float MaxHeight = 0.7f;

        [SerializeField]
        [Range(0, 1)]
        float SmallTerminalRatio = 0.33f;

        [Range(100, 1000)]
        [SerializeField]
        float ToggleSpeed = 1000;

        [SerializeField] string ToggleHotkey      = "`";
        [SerializeField] string ToggleFullHotkey  = "#`";
        [SerializeField] int BufferSize           = 100;

        [Header("Input")]
        [SerializeField] Font ConsoleFont;
        [SerializeField] string InputCaret        = ">";
        [SerializeField] bool ShowGUIButtons;
        [SerializeField] bool RightAlignButtons;

        [Header("Theme")]
        [Range(0, 1)]
        [SerializeField] float InputContrast;
        [Range(0, 1)]
        [SerializeField] float InputAlpha         = 0.5f;

        [SerializeField] Color BackgroundColor    = Color.black;
        [SerializeField] Color ForegroundColor    = Color.white;
        [SerializeField] Color ShellColor         = Color.white;
        [SerializeField] Color InputColor         = Color.cyan;
        [SerializeField] Color WarningColor       = Color.yellow;
        [SerializeField] Color ErrorColor         = Color.red;

        private static UdpClient replSocket;
        IFn log;
        //can't set *ns* with eval! so pjago.repl keeps track of the *ns* binding
        //is almost a reducing function, but in the form (env str) -> {:res r :env e}
        IFn repl_rfn;
        IPersistentMap map;
        Keyword result;
        Keyword env;

        TerminalReplState state;
        public TextEditor editor_state;
        bool input_fix;
        bool move_cursor;
        public bool carret_return;
        bool initial_open; // Used to focus on TextField when console opens
        Rect window;
        float current_open_t;
        float open_target;
        float real_window_size;
        string command_text;
        string cached_command_text;
        string last_command_text;
        Vector2 scroll_position;
        GUIStyle window_style;
        GUIStyle label_style;
        GUIStyle input_style;
        Texture2D background_texture;
        Texture2D input_background_texture;

        public static CommandLog Buffer { get; private set; }
        public static CommandShell Shell { get; private set; }
        public static CommandHistory History { get; private set; }
        public static CommandAutocomplete Autocomplete { get; private set; }

        static TerminalRepl ()
        {
            // Util.require("arcadia.core");
            // Util.require("pjago.repl");
        }

        public void Update ()
        {
            if (replSocket != null)
               RT.var("pjago.repl", "eval-queue").invoke();
        }

        public static bool IssuedError {
            get { return Shell.IssuedErrorMessage != null; }
        }

        public bool IsClosed {
            get { return state == TerminalReplState.Close && Mathf.Approximately(current_open_t, open_target); }
        }

        public static void Log(string format, params object[] message) {
            Log(TerminalLogType.ShellMessage, format, message);
        }

        public static void Log(TerminalLogType type, string format, params object[] message) {
            Buffer.HandleLog(string.Format(format, message), type);
        }

        public void SetState(TerminalReplState new_state) {
            input_fix = true;
            cached_command_text = command_text;
            command_text = "";

            switch (new_state) {
                case TerminalReplState.Close: {
                    open_target = 0;
                    break;
                }
                case TerminalReplState.OpenSmall: {
                    open_target = Screen.height * MaxHeight * SmallTerminalRatio;
                    if (current_open_t > open_target) {
                        // Prevent resizing from OpenFull to OpenSmall if window y position
                        // is greater than OpenSmall's target
                        open_target = 0;
                        state = TerminalReplState.Close;
                        return;
                    }
                    real_window_size = open_target;
                    scroll_position.y = int.MaxValue;
                    break;
                }
                case TerminalReplState.OpenFull:
                default: {
                    real_window_size = Screen.height * MaxHeight;
                    open_target = real_window_size;
                    break;
                }
            }

            state = new_state;
        }

        public void ToggleState(TerminalReplState new_state) {
            if (state == new_state) {
                SetState(TerminalReplState.Close);
            } else {
                SetState(new_state);
            }
        }

        void OnEnable() {
            Buffer = new CommandLog(BufferSize);
            Shell = new CommandShell();
            History = new CommandHistory();
            Autocomplete = new CommandAutocomplete();

            // Hook Unity log events
            Application.logMessageReceived += HandleUnityLog;
        }

        void OnDisable() {
            Application.logMessageReceived -= HandleUnityLog;
        }

        void Start() {
            Util.require("arcadia.core");
            Util.require("pjago.repl");

            carret_return = false;
            replSocket = (UdpClient)RT.var("pjago.repl", "start-server").invoke(11211);
            log = RT.var("arcadia.core", "log");
            repl_rfn = RT.var("pjago.repl", "repl-eval-print");
            map = (IPersistentMap) RT.var("pjago.repl", "env-map").invoke();
            result = Keyword.intern(Symbol.create("result"));
            env = Keyword.intern(Symbol.create("env"));

            if (ConsoleFont == null) {
                ConsoleFont = Font.CreateDynamicFontFromOSFont("Courier New", 16);
                Debug.LogWarning("Command Console Warning: Please assign a font.");
            }

            command_text = "";
            cached_command_text = command_text;
            last_command_text = command_text;
            Assert.AreNotEqual(ToggleHotkey.ToLower(), "return", "Return is not a valid ToggleHotkey");

            SetupWindow();
            SetupInput();
            SetupLabels();

            Shell.RegisterCommands();

            if (IssuedError) {
                Log(TerminalLogType.Error, "Error: {0}", Shell.IssuedErrorMessage);
            }

            foreach (var command in Shell.Commands) {
                Autocomplete.Register(command.Key);
            }

            if (replSocket != null)
               Log("REPL is listening on " + replSocket.Client.LocalEndPoint);

            Debug.Log("Arcadia Started!");
        }

        void OnGUI() {
            if (Event.current.Equals(Event.KeyboardEvent(ToggleHotkey))) {
                SetState(TerminalReplState.OpenSmall);
                initial_open = true;
            } else if (Event.current.Equals(Event.KeyboardEvent(ToggleFullHotkey))) {
                SetState(TerminalReplState.OpenFull);
                initial_open = true;
            }

            if (ShowGUIButtons) {
                DrawGUIButtons();
            }

            if (IsClosed) {
                return;
            }

            HandleOpenness();
            window = GUILayout.Window(88, window, DrawConsole, "", window_style);
        }

        void SetupWindow() {
            real_window_size = Screen.height * MaxHeight / 3;
            window = new Rect(0, current_open_t - real_window_size, Screen.width, real_window_size);

            // Set background color
            background_texture = new Texture2D(1, 1);
            background_texture.SetPixel(0, 0, BackgroundColor);
            background_texture.Apply();

            window_style = new GUIStyle();
            window_style.normal.background = background_texture;
            window_style.padding = new RectOffset(4, 4, 4, 4);
            window_style.normal.textColor = ForegroundColor;
            window_style.font = ConsoleFont;
        }

        void SetupLabels() {
            label_style = new GUIStyle();
            label_style.font = ConsoleFont;
            label_style.normal.textColor = ForegroundColor;
            label_style.wordWrap = true;
        }

        void SetupInput() {
            input_style = new GUIStyle();
            input_style.padding = new RectOffset(4, 4, 4, 4);
            input_style.font = ConsoleFont;
            input_style.fixedHeight = ConsoleFont.fontSize * 1.6f;
            input_style.normal.textColor = InputColor;

            var dark_background = new Color();
            dark_background.r = BackgroundColor.r - InputContrast;
            dark_background.g = BackgroundColor.g - InputContrast;
            dark_background.b = BackgroundColor.b - InputContrast;
            dark_background.a = InputAlpha;

            input_background_texture = new Texture2D(1, 1);
            input_background_texture.SetPixel(0, 0, dark_background);
            input_background_texture.Apply();
            input_style.normal.background = input_background_texture;
        }

        void DrawConsole(int Window2D) {
            GUILayout.BeginVertical();

            // change scroll_position based on mouse scroll here
            // apparently this code is already there
            // if (GUI.GetNameOfFocusedControl() == "command_text_field") {
            //     float scroll_wheel = Input.GetAxis("Mouse ScrollWheel");
            //     if (scroll_wheel < 0f)
            //         scroll_position.y = scroll_position.y + 10;
            //     else if (scroll_wheel > 0f)
            //         scroll_position.y = scroll_position.y - 10;
            // }

            scroll_position = GUILayout.BeginScrollView(scroll_position, false, false, GUIStyle.none, GUIStyle.none);
            GUILayout.FlexibleSpace();
            DrawLogs();
            GUILayout.EndScrollView();

            if (move_cursor) {
                CursorToEnd();
                move_cursor = false;
                carret_return = false;
            }

            if (Event.current.Equals(Event.KeyboardEvent("escape"))) {
                SetState(TerminalReplState.Close);
            }
            if (Event.current.Equals(Event.KeyboardEvent("return"))) {
                EnterCommand();
                initial_open = true;
                carret_return = true;
            } else if (Event.current.Equals(Event.KeyboardEvent("up"))) {
                command_text = History.Previous();
                move_cursor = true;
            } else if (Event.current.Equals(Event.KeyboardEvent("down"))) {
                command_text = History.Next();
            } else if (IsClosed && Event.current.Equals(Event.KeyboardEvent(ToggleHotkey))) {
                SetState(TerminalReplState.OpenSmall);
            } else if (IsClosed && Event.current.Equals(Event.KeyboardEvent(ToggleFullHotkey))) {
                SetState(TerminalReplState.OpenFull);
            }
            // else if (Event.current.Equals(Event.KeyboardEvent("tab"))) {
            //     CompleteCommand();
            //     move_cursor = true; // Wait till next draw call
            // }

            GUILayout.BeginHorizontal();

            if (InputCaret != "") {
                GUILayout.Label(InputCaret, input_style, GUILayout.Width(ConsoleFont.fontSize));
            }

            GUI.SetNextControlName("command_text_field");
            command_text = GUILayout.TextField(command_text, input_style);

            if (input_fix && command_text.Length > 0) {
                command_text = cached_command_text; // Otherwise the TextField picks up the ToggleHotkey character event
                input_fix = false;                  // Prevents checking string Length every draw call
            }

            if (initial_open) {
                GUI.FocusControl("command_text_field");
                initial_open = false;
            }

            if (carret_return) {
                GUI.FocusControl("");
                move_cursor = true;
                initial_open = true;
            }

            if (ShowGUIButtons && GUILayout.Button("| run", input_style, GUILayout.Width(Screen.width / 10))) {
                EnterCommand();
                carret_return = true;
            }

            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
        }

        void DrawLogs() {
            foreach (var log in Buffer.Logs) {
                label_style.normal.textColor = GetLogColor(log.type);
                GUILayout.TextField(log.message, label_style); // non modifiable TextField
            }
        }

        void DrawGUIButtons() {
            int size = ConsoleFont.fontSize;
            float x_position = RightAlignButtons ? Screen.width - 7 * size : 0;

            // 7 is the number of chars in the button plus some padding, 2 is the line height.
            // The layout will resize according to the font size.
            GUILayout.BeginArea(new Rect(x_position, current_open_t, 7 * size, size * 2));
            GUILayout.BeginHorizontal();

            if (GUILayout.Button("Small", window_style)) {
                ToggleState(TerminalReplState.OpenSmall);
            } else if (GUILayout.Button("Full", window_style)) {
                ToggleState(TerminalReplState.OpenFull);
            }

            GUILayout.EndHorizontal();
            GUILayout.EndArea();
        }

        void HandleOpenness() {
            float dt = ToggleSpeed * 0.02f;

            if (state == TerminalReplState.OpenFull)
                dt = dt * 1.0f/SmallTerminalRatio;

            if (current_open_t < open_target) {
                current_open_t += dt;
                if (current_open_t > open_target) current_open_t = open_target;
            } else if (current_open_t > open_target) {
                current_open_t -= dt;
                if (current_open_t < open_target) current_open_t = open_target;
            } else {
                if (input_fix) {
                    input_fix = false;
                }
                return; // Already at target
            }

            window = new Rect(0, current_open_t - real_window_size, Screen.width, real_window_size);
        }

        void EnterCommand() {
            Log(TerminalLogType.Input, "{0}", command_text);
            // Shell.RunCommand(command_text);

            map = (IPersistentMap) repl_rfn.invoke(map.valAt(env), command_text);
            string txt = (string) map.valAt(result);
            txt = txt.TrimEnd();
            log.invoke(txt);
            if(command_text != last_command_text) {
                History.Push(command_text);
            }
            else {
                History.Next();
            }
            last_command_text = command_text;
            if (IssuedError) {
                Log(TerminalLogType.Error, "Error: {0}", Shell.IssuedErrorMessage);
            }

            command_text = "";
            scroll_position.y = int.MaxValue;
        }

        void CompleteCommand() {
            string head_text = command_text;
            int format_width = 0;

            string[] completion_buffer = Autocomplete.Complete(ref head_text, ref format_width);
            int completion_length = completion_buffer.Length;

            if (completion_length != 0) {
                command_text = head_text;
            }

            if (completion_length > 1) {
                // Print possible completions
                var log_buffer = new StringBuilder();

                foreach (string completion in completion_buffer) {
                    log_buffer.Append(completion.PadRight(format_width + 4));
                }

                Log("{0}", log_buffer);
                move_cursor = true;
                scroll_position.y = int.MaxValue;
            }
        }

        void CursorToEnd() {
            editor_state = (TextEditor)GUIUtility.GetStateObject(typeof(TextEditor), GUIUtility.keyboardControl);
            editor_state.MoveTextEnd();
        }

        void HandleUnityLog(string message, string stack_trace, LogType type) {
            Buffer.HandleLog(message, stack_trace, (TerminalLogType)type);
            scroll_position.y = int.MaxValue;
        }

        Color GetLogColor(TerminalLogType type) {
            switch (type) {
                case TerminalLogType.Message: return ForegroundColor;
                case TerminalLogType.Warning: return WarningColor;
                case TerminalLogType.Input: return InputColor;
                case TerminalLogType.ShellMessage: return ShellColor;
                default: return ErrorColor;
            }
        }
    }
}
