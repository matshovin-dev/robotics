/*
 * wb_plotter.c - Enkel y(t) graf-plotter
 *
 * Bruk: ./wb_plotter > output.svg
 *       Åpne SVG-filen i nettleser for å se grafen
 */

#include <stdio.h>
#include <math.h>

// ============ KONFIGURASJON ============

// Tidsintervall
#define T_START 0.0
#define T_END 2.0
#define T_STEP 0.01

// Graf-størrelse (piksler)
#define WIDTH 800
#define HEIGHT 400
#define MARGIN 50

// ============ DEFINER DINE FUNKSJONER HER ============

// Graf 1: Hovedfunksjon
double y1(double t)
{
	return 2.1 * sin(23.0 * M_PI * t);
}

// Graf 2: Sekundær funksjon (plottes oppå graf 1)
double y2(double t)
{
	return 1.0 * cos(1.0 * M_PI * t);
}

// ============ PLOTTER-KODE ============

typedef struct {
	double (*func)(double);
	const char *color;
	const char *name;
} Graph;

void find_y_range(Graph *graphs, int n_graphs, double *y_min, double *y_max)
{
	*y_min = 1e9;
	*y_max = -1e9;

	for (int g = 0; g < n_graphs; g++) {
		for (double t = T_START; t <= T_END; t += T_STEP) {
			double y = graphs[g].func(t);
			if (y < *y_min)
				*y_min = y;
			if (y > *y_max)
				*y_max = y;
		}
	}

	// Litt margin
	double range = *y_max - *y_min;
	*y_min -= range * 0.1;
	*y_max += range * 0.1;
}

double map_t_to_x(double t)
{
	return MARGIN +
	       (t - T_START) / (T_END - T_START) * (WIDTH - 2 * MARGIN);
}

double map_y_to_screen(double y, double y_min, double y_max)
{
	return HEIGHT - MARGIN -
	       (y - y_min) / (y_max - y_min) * (HEIGHT - 2 * MARGIN);
}

void print_svg_header(void)
{
	printf("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
	printf("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\">\n",
	       WIDTH, HEIGHT);
	printf("<rect width=\"100%%\" height=\"100%%\" fill=\"white\"/>\n");
}

void print_axes(double y_min, double y_max)
{
	// Bakgrunn for plot-område
	printf("<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"#f8f8f8\" stroke=\"#ccc\"/>\n",
	       MARGIN, MARGIN, WIDTH - 2 * MARGIN, HEIGHT - 2 * MARGIN);

	// X-akse (y=0 hvis synlig)
	if (y_min <= 0 && y_max >= 0) {
		double y0_screen = map_y_to_screen(0, y_min, y_max);
		printf("<line x1=\"%d\" y1=\"%.1f\" x2=\"%d\" y2=\"%.1f\" stroke=\"#888\" stroke-width=\"1\"/>\n",
		       MARGIN, y0_screen, WIDTH - MARGIN, y0_screen);
	}

	// Aksetekst
	printf("<text x=\"%d\" y=\"%d\" font-size=\"12\" text-anchor=\"middle\">t = %.1f</text>\n",
	       MARGIN, HEIGHT - 10, T_START);
	printf("<text x=\"%d\" y=\"%d\" font-size=\"12\" text-anchor=\"middle\">t = %.1f</text>\n",
	       WIDTH - MARGIN, HEIGHT - 10, T_END);
	printf("<text x=\"%d\" y=\"%d\" font-size=\"12\" text-anchor=\"end\">%.2f</text>\n",
	       MARGIN - 5, MARGIN + 5, y_max);
	printf("<text x=\"%d\" y=\"%d\" font-size=\"12\" text-anchor=\"end\">%.2f</text>\n",
	       MARGIN - 5, HEIGHT - MARGIN + 5, y_min);
}

void print_graph(Graph *graph, double y_min, double y_max)
{
	printf("<path d=\"M");

	int first = 1;
	for (double t = T_START; t <= T_END; t += T_STEP) {
		double x = map_t_to_x(t);
		double y = map_y_to_screen(graph->func(t), y_min, y_max);

		if (first) {
			printf("%.1f,%.1f", x, y);
			first = 0;
		} else {
			printf(" L%.1f,%.1f", x, y);
		}
	}

	printf("\" fill=\"none\" stroke=\"%s\" stroke-width=\"2\"/>\n",
	       graph->color);
}

void print_legend(Graph *graphs, int n_graphs)
{
	int x = WIDTH - MARGIN - 100;
	int y = MARGIN + 20;

	for (int i = 0; i < n_graphs; i++) {
		printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"%s\" stroke-width=\"2\"/>\n",
		       x, y + i * 20, x + 20, y + i * 20, graphs[i].color);
		printf("<text x=\"%d\" y=\"%d\" font-size=\"12\">%s</text>\n",
		       x + 25, y + i * 20 + 4, graphs[i].name);
	}
}

void print_svg_footer(void)
{
	printf("</svg>\n");
}

int main(void)
{
	// ============ SETT OPP GRAFENE HER ============
	Graph graphs[] = {
		{ y1, "#2196F3", "y = 0.3*sin(2*pi*t)" },  // Blå
		{ y2, "#F44336", "y = 2*cos(2*pi*t)" },	 // Rød
	};
	int n_graphs = sizeof(graphs) / sizeof(graphs[0]);

	// Finn y-range
	double y_min, y_max;
	find_y_range(graphs, n_graphs, &y_min, &y_max);

	// Generer SVG
	print_svg_header();
	print_axes(y_min, y_max);

	// Plot alle grafene
	for (int i = 0; i < n_graphs; i++) {
		print_graph(&graphs[i], y_min, y_max);
	}

	print_legend(graphs, n_graphs);
	print_svg_footer();

	return 0;
}
