#include <Novice.h>
#include <MyMath.h>
#include <imgui.h>

const char kWindowTitle[] = "LE2A_08_スヤマハナ";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	float kWindowWidth = 1280;
	float kWindowHeight = 720;

	Vector3 rotate{0,5,0};
	Vector3 translate{};
	Vector3 cameraPosition{ 0, 0, -20 };
	Vector3 kLocalVertices[3]{};

	Triangle triangle{};
	Segment segment{ {-2.0f, -1.0f, 0.0f}, {3.0f, 2.0f, 2.0f} };

	triangle.LocalVertices[0] = { 0,1,0 };
	triangle.LocalVertices[1] = { 2,-2,0 };
	triangle.LocalVertices[2] = { -2,-2,0 };

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		//各種行列の計算
		Matrix4x4 worldMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, rotate, translate);
		Matrix4x4 cameraMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, cameraPosition);
		Matrix4x4 viewMatrix = Inverse(cameraMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		Matrix4x4 worldViewProjectionMatrix = Multiply(worldMatrix, Multiply(viewMatrix, projectionMatrix));
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		Vector3 start = Transform(Transform(segment.origin, worldViewProjectionMatrix), viewportMatrix);
		Vector3 end = Transform(Transform(Add(segment.origin, segment.diff), worldViewProjectionMatrix), viewportMatrix);

		ImGui::Begin("ImGui");
		ImGui::SliderFloat3("Segment Origin", reinterpret_cast<float*>(&segment.origin), -5, 5);
		ImGui::SliderFloat3("Segment Diff", reinterpret_cast<float*>(&segment.diff), -5, 5);
		ImGui::SliderFloat3("Triangle Vertices1", reinterpret_cast<float*>(&triangle.LocalVertices[0]), -2, 2);
		ImGui::SliderFloat3("Triangle Vertices2", reinterpret_cast<float*>(&triangle.LocalVertices[1]), -2, 2);
		ImGui::SliderFloat3("Triangle Vertices3", reinterpret_cast<float*>(&triangle.LocalVertices[2]), -2, 2);
		ImGui::SliderFloat3("Camera Rotate", reinterpret_cast<float*>(&rotate), -10, 10);
		ImGui::SliderFloat3("Camera Translate", reinterpret_cast<float*>(&translate), -10, 10);

		ImGui::End();

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		DrawLineTriangle(triangle, worldViewProjectionMatrix, viewportMatrix, WHITE);
		if (IsCollision(triangle, segment)) {
			Novice::DrawLine(int(start.x), int(start.y), int(end.x), int(end.y), RED);
		} else {
			Novice::DrawLine(int(start.x), int(start.y), int(end.x), int(end.y), WHITE);
		}
		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
