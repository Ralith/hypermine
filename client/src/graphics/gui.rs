use yakui::{align, colored_box, Alignment, Color};

pub struct GuiState {
    show_gui: bool,
}

impl GuiState {
    pub fn new() -> Self {
        GuiState { show_gui: true }
    }

    /// Toggles whether the GUI is shown
    pub fn toggle_gui(&mut self) {
        self.show_gui = !self.show_gui;
    }

    /// Prepare the GUI for rendering. This should be called between
    /// Yakui::start and Yakui::finish.
    pub fn run(&self) {
        if !self.show_gui {
            return;
        }

        align(Alignment::CENTER, || {
            colored_box(Color::BLACK.with_alpha(0.9), [5.0, 5.0]);
        });
    }
}
