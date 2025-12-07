#include "song_lib.h"
#include "../vendor/cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Global song library */
struct song song_lib[SONG_LIB_SIZE];
int song_lib_count = 0;

void song_lib_init(void)
{
	memset(song_lib, 0, sizeof(song_lib));
	song_lib_count = 0;
}

int song_lib_add(const char *name, const char *wav_path, float bpm,
		 float master_phase)
{
	if (song_lib_count >= SONG_LIB_SIZE)
		return -1;

	struct song *s = &song_lib[song_lib_count];

	strncpy(s->name, name, SONG_NAME_MAX - 1);
	s->name[SONG_NAME_MAX - 1] = '\0';

	strncpy(s->wav_path, wav_path, SONG_PATH_MAX - 1);
	s->wav_path[SONG_PATH_MAX - 1] = '\0';

	s->bpm = bpm;
	s->master_phase = master_phase;

	return song_lib_count++;
}

struct song *song_lib_get(int index)
{
	if (index < 0 || index >= song_lib_count)
		return NULL;
	return &song_lib[index];
}

int song_lib_find(const char *name)
{
	for (int i = 0; i < song_lib_count; i++) {
		if (strcmp(song_lib[i].name, name) == 0)
			return i;
	}
	return -1;
}

int song_lib_save(const char *path)
{
	cJSON *root = cJSON_CreateObject();
	cJSON *songs = cJSON_CreateArray();

	for (int i = 0; i < song_lib_count; i++) {
		cJSON *song = cJSON_CreateObject();
		cJSON_AddStringToObject(song, "name", song_lib[i].name);
		cJSON_AddStringToObject(song, "wav_path", song_lib[i].wav_path);
		cJSON_AddNumberToObject(song, "bpm", song_lib[i].bpm);
		cJSON_AddNumberToObject(song, "master_phase",
					song_lib[i].master_phase);
		cJSON_AddItemToArray(songs, song);
	}

	cJSON_AddItemToObject(root, "songs", songs);

	char *json_str = cJSON_Print(root);
	cJSON_Delete(root);

	if (!json_str)
		return -1;

	FILE *f = fopen(path, "w");
	if (!f) {
		free(json_str);
		return -1;
	}

	fprintf(f, "%s\n", json_str);
	fclose(f);
	free(json_str);

	return 0;
}

int song_lib_load(const char *path)
{
	FILE *f = fopen(path, "r");
	if (!f)
		return -1;

	fseek(f, 0, SEEK_END);
	long size = ftell(f);
	fseek(f, 0, SEEK_SET);

	char *json_str = malloc(size + 1);
	if (!json_str) {
		fclose(f);
		return -1;
	}

	fread(json_str, 1, size, f);
	json_str[size] = '\0';
	fclose(f);

	cJSON *root = cJSON_Parse(json_str);
	free(json_str);

	if (!root)
		return -1;

	song_lib_init();

	cJSON *songs = cJSON_GetObjectItem(root, "songs");
	if (!cJSON_IsArray(songs)) {
		cJSON_Delete(root);
		return -1;
	}

	cJSON *song;
	cJSON_ArrayForEach(song, songs) {
		cJSON *name = cJSON_GetObjectItem(song, "name");
		cJSON *wav_path = cJSON_GetObjectItem(song, "wav_path");
		cJSON *bpm = cJSON_GetObjectItem(song, "bpm");
		cJSON *phase = cJSON_GetObjectItem(song, "master_phase");

		if (cJSON_IsString(name) && cJSON_IsString(wav_path) &&
		    cJSON_IsNumber(bpm) && cJSON_IsNumber(phase)) {
			song_lib_add(name->valuestring, wav_path->valuestring,
				     (float)bpm->valuedouble,
				     (float)phase->valuedouble);
		}
	}

	cJSON_Delete(root);
	return song_lib_count;
}
