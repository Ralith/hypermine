use yakui::{
    align, colored_box, colored_box_container, label, pad, widgets::Pad, Alignment, Color,
};

use crate::Sim;

pub struct GuiState {
    show_gui: bool,
}

impl GuiState {
    pub fn new() -> Self {
        GuiState { show_gui: true }
    }

    pub fn toggle_gui(&mut self) {
        self.show_gui = !self.show_gui;
    }
}

pub fn gui(gui_state: &GuiState, sim: &Sim) {
    if !gui_state.show_gui {
        return;
    }

    align(Alignment::CENTER, || {
        colored_box(Color::BLACK.with_alpha(0.9), [5.0, 5.0]);
    });

    align(Alignment::TOP_LEFT, || {
        pad(Pad::all(8.0), || {
            colored_box_container(Color::BLACK.with_alpha(0.7), || {
                label(format!("Selected material: {:?}", sim.selected_material()));
            });
        });
    });
}
