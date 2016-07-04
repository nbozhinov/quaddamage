#include "render.h"
#include <thread>

const char* DEFAULT_SCENE = "data/emptyScene.qdmg";

int main ( int argc, char** argv )
{

	initRandom(42);
	Color::init_sRGB_cache();
	const char* sceneFile = argc == 2 ? argv[1] : DEFAULT_SCENE;
	if (!scene.parseScene(sceneFile)) {
		printf("Could not parse the scene!\n");
		return -1;
	}

	initGraphics(scene.settings.frameWidth, scene.settings.frameHeight,
		scene.settings.interactive && scene.settings.fullscreen);

	setWindowCaption("Quad Damage: preparing...");

	if (scene.settings.numThreads == 0)
		scene.settings.numThreads = std::thread::hardware_concurrency();

	pool.preload_threads(scene.settings.numThreads);

	scene.beginRender();

	if (scene.settings.interactive) {
		mainloop(sceneFile);
	} else {

		setWindowCaption("Quad Damage: rendering...");
		Uint32 startTicks = SDL_GetTicks();
		renderScene_threaded();
		Uint32 elapsedMs = SDL_GetTicks() - startTicks;
		printf("Render took %.2fs\n", elapsedMs / 1000.0f);
		setWindowCaption("Quad Damage: rendered in %.2fs\n", elapsedMs / 1000.0f);

		displayVFB(vfb);
		waitForUserExit();
	}
	closeGraphics();
	printf("Exited cleanly\n");
	return 0;
}
