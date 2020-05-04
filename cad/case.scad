$fs = 0.1;

bolt_centres = [
	[-47.5, -14.0, 0],
	[ 47.5, -14.0, 0],
	[-47.5,  14.0, 0],
	[ 47.5,  14.0, 0],
];

led_centres = [
	[-33, -11.4, 0],
	[ -3, -3, 0],
];

board_t = 1.6;
module board() {
	translate([0, 0, -(0)]) {
		translate([0, 0, -20]) linear_extrude(height=20) import("board.dxf", layer="battholder");
	}
	linear_extrude(height=board_t) import("board.dxf", layer="outline");
	translate([0, 0, board_t]) {
		linear_extrude(height=3.6) import("board.dxf", layer="wifi_ant");
		linear_extrude(height=2.7) import("board.dxf", layer="usb");
		linear_extrude(height=2) import("board.dxf", layer="buttons");
	}
}

module bottom() {
	difference() {
		union() {
			translate([0, 0, -(/*board_t +*/ 4)]) linear_extrude(height=4) import("board.dxf", layer="case_bottom_wall_pads");
			translate([0, 0, 0]) linear_extrude(height=3) import("board.dxf", layer="case_bottom_wall");
			translate([0, 0, 3]) linear_extrude(height=1) import("board.dxf", layer="case_bottom_lip");
			difference() {
				translate([0, 0, -5.5]) linear_extrude(height=1.6) import("board.dxf", layer="case_outline");
				for (b = led_centres) {
					translate(b + [0, 0, -(5.5) + 10.3]) sphere(r = 10);
				}
			}
		}

		union() {
			for (b = bolt_centres) {
				translate(b + [0, 0, -(5.6)]) cylinder(r = 2.4, h = 4.0);
				translate(b + [0, 0, -(5.6)]) cylinder(r = 1, h = 14.0);
			}

			rotate([180, 0, 0]) intersection() {
				union() {
					translate([0, 0, -(3) + 1]) linear_extrude(height=6) import("board.dxf", layer="btn_outer_sub");
					translate([0, 0, -(3) + 1]) linear_extrude(height=6) import("board.dxf", layer="btn_inner_sub");
				}
				translate([0, 0, 1]) rotate([0, -90, 0]) translate([0, 0, -10]) linear_extrude(height=50) import("board.dxf", layer="btn_ortho");
			}
			rotate([180, 0, 0]) translate([0, 0, 0]) {
				translate([0, 4, 0]) linear_extrude(height=2.7) import("board.dxf", layer="usb");
			}
		}
	}

	for (i = [0:2]) {
		translate([-27.3 + (9.5 * i), -19.2, -(1)]) rotate([90, 0, 0]) rotate_extrude() import("board.dxf", layer="button_nubbin");
	}
}

module top() {
	difference() {
		union() {
			translate([0, 0, 3]) linear_extrude(height=1) import("board.dxf", layer="case_top_lip");
			translate([0, 0, 3 + 1]) linear_extrude(height=22-4) import("board.dxf", layer="case_top_wall");
			translate([0, 0, 22]) linear_extrude(height=1.6) import("board.dxf", layer="case_outline");

			for (b = bolt_centres) {
				translate(b + [0, 0, board_t + 0.2]) cylinder(r = 3, h = 4.0);
			}
		}

		union() {
			for (b = bolt_centres) {
				translate(b + [0, 0, board_t + 1.6]) rotate([0, 0, 60]) cylinder(r = 2.5, h = 140, $fn=6);
				translate(b + [0, 0, 0]) cylinder(r = 1, h = 140);
			}

			translate([-39.5, 15, 12]) rotate([-90, 0, 0]) cylinder(r = 3.5, h = 10);
		}
	}
}

bottom();
translate([0, 50, 14]) rotate([180, 0, 0]) translate([0, 0, -4]) top();

//translate([0, 0, board_t]) rotate([180, 0, 0]) board();


