ÖDEV 2
# RectangleArea

RectangleArea, bir dikdörtgenin alanını hesaplamak için yapıyoruz. Kullanıcıdan dikdörtgenin uzunluğunu ve genişliğini alarak alanı hesaplar ve sonucu ekrana yazdırır. 

Özellikler

- Kullanıcıdan uzunluk ve genişlik girişi alır.
- Dikdörtgenin alanını hesaplar.
- Sonucu kullanıcıya gösterir.

Kullanım
	Önce ROS MASTER'ı çalıştırın. Bunun için terminale aşağıdaki komutu yazınız;
		roscore
	Daha sonra projeyi çalıştırmak için rectangle_area_server.py python kodunu açacağız. Bunun içinde yeni bir terminal açarak aşağıdaki komutu giriniz;
		rosrun ros_tutorials rectangle_area_server.py
	Sonrasında kullanıcıdan girdi alacağımız kısım için yeni bir terminal açarak aşağıdaki komutu giriyoruz;
	[x] [y] yazılan yere istenilen iki float değerimizi giriyoruz.
		rosrun ros_tutorials rectangle_area_client.py [x] [y] 
		[x] [y]
	
	Ekranda dikdörtgenin alanı gösterilecektir.

Bu proje, ROS'un hizmet (service) mimarisini kullanır:

    Server (Sunucu): rectangle_area_server.py, kullanıcıdan gelen istekleri dinler ve alan hesaplama işlemini gerçekleştirir. Kullanıcıdan aldığı uzunluk ve genişlik değerlerini kullanarak dikdörtgenin alanını hesaplar. Hesaplama işlemi tamamlandığında, sonucu geri döner.

    Client (İstemci): rectangle_area_client.py, kullanıcıdan uzunluk ve genişlik değerlerini alır ve bu değerleri sunucuya gönderir. Sunucudan gelen yanıtı alarak sonucu ekrana yazdırır.
    
ARKA PLAN
İstenilen değerleri içeren srv dosyası oluşturduk bunlar float32 width, float32 height, float32 area.
Bu oluşturduğumuz srv dosyasını CMakeList'e add_service kısmına ekliyoruz.
Ekledikten sonra bir dosya oluşturduk ve dosyanın içine python kodlarımız bulunan dosyaları attık.
Attıktan sonra içlerini düzenleyerek istenilen RectangleArea yapmış olduk.
