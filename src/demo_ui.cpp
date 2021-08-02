#include "demo_ui.h"

#include "stereokit.h"
#include "stereokit_ui.h"
#include "stdio.h"
using namespace sk;

///////////////////////////////////////////

sprite_t ui_sprite;

///////////////////////////////////////////

void demo_ui_init() {
	ui_sprite = sprite_create_file("StereoKitWide.png", sprite_type_single);
}

float val = 0.5f;
float val2 = 0.5f;

float temp_val;
float temp_val2;
bool active = false;

///////////////////////////////////////////

void demo_ui_update() {
	const sk::hand_t * hand_right = sk::input_hand(sk::handed_right);
	if (hand_right->tracked_state == button_state_inactive) {
		return;
	}

	sk::vec3 head_to_hand;
	head_to_hand = hand_right->palm.position - sk::input_head()->position;
	printf("%f %f %f\n", head_to_hand.x, head_to_hand.y, head_to_hand.z);

	// sk::vec3 palm_looking_at = sk::vec3(hand_right->palm.orientation);

	hierarchy_push(sk::pose_matrix(hand_right->palm));

	hierarchy_pop();

}

// void demo_ui_update() {

// 	// input_hand(handed_right).root;
// 	hierarchy_push(sk::pose_matrix(sk::input_hand(sk::handed_right)->palm));
// 	pose_t window_pose = //pose_t{ {0,0.5f,0},{0,0,0,1} };
// 		pose_t{ {0.10,0.,0.0f}, quat_identity };
// 	ui_window_begin("", window_pose, vec2{val,val2}, sk::ui_win_empty);

// 	static char buffer[128] = {};
// 	ui_input("text", buffer, 128, {16*cm2m,ui_line_height()});
// 	active = false;
// 	bool active2 = false;
// 	active = ui_hslider("slider", temp_val, 0, 1, 0, 72*mm2m); ui_sameline();
// 	active2 = ui_hslider("slider2", temp_val2, 0, 1, 0, 72*mm2m);
// 	active = active || active2;
// 	if (input_key(key_mouse_left) & button_state_active)
// 		ui_image(ui_sprite, vec2{ 6,0 }*cm2m);
// 	if (ui_button("Press me!")) {
// 		ui_button("DYNAMIC BUTTON!!");
// 	}


// 	ui_window_end();
// 	printf("%i\n", active);
// 	if (!active){
// 		printf("Okay!\n");
// 		val = temp_val;
// 		val2 = temp_val2;
// 	}
// 	hierarchy_pop();
// }

///////////////////////////////////////////

void demo_ui_shutdown() {
	// Release everything
	sprite_release(ui_sprite);
}
