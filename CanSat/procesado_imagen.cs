using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

Image<Rgba32> CreateAnaglyph(Image<L8> left, Image<L8> right)
{
    int width = left.Width;
    int height = left.Height;

    var output = new Image<Rgba32>(width, height);

    output.ProcessPixelRows(left, right, (outAcc, leftAcc, rightAcc) =>
    {
        for (int row = 0; row < outAcc.Height; row++)
        {
            var outRow = outAcc.GetRowSpan(row);
            var leftRow = leftAcc.GetRowSpan(row);
            var rightRow = rightAcc.GetRowSpan(row);

            for (int col = 0; col < outRow.Length; col++)
            {
                byte l = leftRow[col].PackedValue;
                byte r = rightRow[col].PackedValue;

                Console.WriteLine($"{l}L - {r}R");
                outRow[col] = new Rgba32(l, r, r);
            }
        }
    });

return output;

}

var left = Image.Load<L8>("left.jpg");
var right = Image.Load<L8>("right.jpg");

var result = CreateAnaglyph(left, right);
result.Save("anaglyph.jpg");