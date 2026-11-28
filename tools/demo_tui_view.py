"""Terminal interface for the paired demonstration workflow."""

import curses
import textwrap
from dataclasses import dataclass

STAGES = ("capture", "home", "replay", "review")
STAGE_NAMES = ("Human take", "Home", "Non-human take", "Review & save")
ENTER_KEYS = ("\n", "\r", curses.KEY_ENTER)
KEY_CAPTIONS = {"enter": "Enter", "space": "Space", "abort": "Esc"}


@dataclass
class Input:
    prompt: str = ""
    task_index: int = 0


def _write(screen, row, column, value, style=0):
    height, width = screen.getmaxyx()
    if row >= height or column >= width - 1:
        return
    try:
        screen.addnstr(row, column, value, width - column - 1, style)
    except curses.error:
        # The terminal can be resized between getmaxyx and the write.
        pass


def _hide_cursor():
    try:
        curses.curs_set(0)
    except curses.error:
        pass


def _colors():
    if not curses.has_colors():
        return curses.A_BOLD, curses.A_BOLD
    try:
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_CYAN, -1)
        curses.init_pair(2, curses.COLOR_YELLOW, -1)
    except curses.error:
        return curses.A_BOLD, curses.A_BOLD
    return curses.color_pair(1) | curses.A_BOLD, curses.color_pair(2) | curses.A_BOLD


def _steps(screen, stage, accent, width):
    current = STAGES.index(stage) if stage in STAGES else len(STAGES)
    labels = [f"{index + 1} {name}" for index, name in enumerate(STAGE_NAMES)]
    if width < 64:
        name = STAGE_NAMES[current] if current < len(STAGES) else "Saved"
        _write(screen, 2, 2, f"{min(current + 1, len(STAGES))}/{len(STAGES)}  {name}", accent)
        return

    column = 2
    for index, label in enumerate(labels):
        if index:
            _write(screen, 2, column, "  /  ", curses.A_DIM)
            column += 5
        style = accent if index == current else curses.A_DIM if index > current else curses.A_NORMAL
        _write(screen, 2, column, label, style)
        column += len(label)


def _footer(controller, pending):
    if pending.prompt == "task":
        if not controller.task_choices:
            return "Esc  Cancel"
        return "Up/Down or J/K  Select    Enter  Apply    Esc  Cancel"
    if pending.prompt == "discard":
        return "Discard this draft and its takes?   Y discard    N keep"
    if pending.prompt == "quit":
        return "Exit this session?   Y exit    N stay"

    commands = [
        f"{KEY_CAPTIONS.get(key, key.upper())}  {caption}"
        for key, caption in controller.actions.items()
        if key != "q"
    ]
    commands.append("Q  Quit")
    return "    ".join(commands)


def _body(controller, alert):
    lines = [(controller.status, curses.A_BOLD)]
    if controller.error:
        lines.append((controller.error, alert))
    if controller.episode_id:
        lines.append((f"Episode  {controller.episode_id}", 0))
    if controller.task:
        lines.append((f"Task   {controller.task}", 0))
    if controller.out_dir:
        lines.append((f"Output {controller.out_dir}", 0))
    lines.extend((detail, curses.A_DIM) for detail in controller.detail)
    return lines


def _task_menu(screen, choices, pending, row, footer_top, accent, alert):
    _write(screen, row, 2, "Choose a task", accent)
    if not choices:
        _write(screen, row + 1, 2, "The manifest has no tasks.", alert)
        return
    selected = min(pending.task_index, len(choices) - 1)
    room = max(1, footer_top - row - 2)
    first = min(max(0, selected - room // 2), max(0, len(choices) - room))
    for index, task in enumerate(choices[first:first + room], first):
        active = index == selected
        prefix = ">" if active else " "
        _write(screen, row + 1 + index - first, 2, f"{prefix} {task}", accent if active else 0)


def _draw(screen, controller, pending, accent, alert):
    height, width = screen.getmaxyx()
    screen.erase()
    if height < 10 or width < 32:
        _write(screen, 0, 0, "Enlarge terminal  |  Q quit", alert)
        screen.refresh()
        return

    _write(screen, 0, 2, controller.title, accent)
    _steps(screen, controller.stage, accent, width)

    footer_lines = textwrap.wrap(_footer(controller, pending), width - 4)
    footer_top = max(6, height - len(footer_lines) - 1)
    row = 4
    if pending.prompt == "task":
        _task_menu(screen, controller.task_choices, pending, row, footer_top, accent, alert)
    for value, style in _body(controller, alert) if pending.prompt != "task" else ():
        for line in textwrap.wrap(value, width - 4):
            if row >= footer_top - 1:
                break
            _write(screen, row, 2, line, style)
            row += 1
        if row >= footer_top - 1:
            break

    _write(screen, footer_top - 1, 2, "─" * (width - 4), curses.A_DIM)
    for row, line in enumerate(footer_lines, footer_top):
        _write(screen, row, 2, line, alert if pending.prompt in {"discard", "quit"} else accent)
    screen.refresh()


def _handle_prompt(controller, pending, character):
    if pending.prompt == "task":
        choices = controller.task_choices
        if character == "\x1b":
            pending.prompt = ""
        elif choices:
            selected = min(pending.task_index, len(choices) - 1)
            if character in (curses.KEY_UP, "k", "K"):
                pending.task_index = max(0, selected - 1)
            elif character in (curses.KEY_DOWN, "j", "J"):
                pending.task_index = min(len(choices) - 1, selected + 1)
            elif character in ENTER_KEYS:
                controller.dispatch("t", choices[selected])
                pending.prompt = ""
        return True
    key = character.lower() if isinstance(character, str) else ""
    if key == "y":
        if pending.prompt == "quit":
            return False
        controller.dispatch("d")
        pending.prompt = ""
    elif key in ("n", "\x1b"):
        pending.prompt = ""
    return True


def _loop(screen, controller):
    screen.keypad(True)
    screen.timeout(150)
    _hide_cursor()
    accent, alert = _colors()
    pending = Input()

    while True:
        controller.refresh()
        _draw(screen, controller, pending, accent, alert)
        try:
            character = screen.get_wch()
        except curses.error:
            continue
        except KeyboardInterrupt:
            character = "\x03"

        if character == curses.KEY_RESIZE:
            continue
        if character in ("\x03", "\x1b") and "abort" in controller.actions:
            pending.prompt = ""
            controller.dispatch("abort")
            continue
        if character == "\x03":
            if pending.prompt:
                pending.prompt = ""
            elif controller.busy or controller.has_draft:
                pending.prompt = "quit"
            else:
                return
            continue
        if pending.prompt:
            if not _handle_prompt(controller, pending, character):
                return
            continue

        if character in ENTER_KEYS:
            key = "enter"
        elif character == " ":
            key = "space"
        else:
            key = character.lower() if isinstance(character, str) else ""
        if key == "q":
            if not (controller.busy or controller.has_draft):
                return
            pending.prompt = "quit"
        elif key in controller.actions:
            if key == "t":
                pending.prompt = "task"
                choices = controller.task_choices
                pending.task_index = choices.index(controller.task) if controller.task in choices else 0
            elif key == "d":
                pending.prompt = "discard"
            elif controller.uses_operator_terminal(key):
                curses.flushinp()
                curses.def_prog_mode()
                curses.endwin()
                try:
                    controller.dispatch(key)
                finally:
                    curses.reset_prog_mode()
                    curses.flushinp()
                    screen.clearok(True)
            else:
                controller.dispatch(key)


def run(controller):
    """Render and control one session using the supplied workflow controller."""
    curses.wrapper(_loop, controller)
