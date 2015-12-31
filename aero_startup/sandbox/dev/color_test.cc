#include "aero_common/colors.h"

int main(int argc, char **argv)
{
  aero::rgb test1_rgb = {142, 181, 72};
  aero::rgb test2_rgb = {219, 195, 117};
  aero::rgb test3_rgb = {248, 203, 118};

  aero::lab test1_lab = aero::colors::rgb2lab(test1_rgb);
  aero::lab test2_lab = aero::colors::rgb2lab(test2_rgb);
  aero::lab test3_lab = aero::colors::rgb2lab(test3_rgb);

  printf("142 181 72 -> %f %f %f\n", test1_lab.l, test1_lab.a, test1_lab.b);
  printf("219 195 117 -> %f %f %f\n", test2_lab.l, test2_lab.a, test2_lab.b);
  printf("248 203 118 -> %f %f %f\n", test3_lab.l, test3_lab.a, test3_lab.b);

  printf("142 181 72 ~ 219 195 117 : %f\n",
	 aero::colors::distance(test1_lab, test2_lab));
  printf("248 203 118 ~ 219 195 117 : %f\n",
	 aero::colors::distance(test3_lab, test2_lab));

  return 0;
}
